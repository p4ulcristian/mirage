#define _GNU_SOURCE
#include "magcal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char *rayneo_magcal_default_path(void)
{
    static char path[512];
    const char *home = getenv("HOME");
    if (!home) home = "/tmp";
    snprintf(path, sizeof path, "%s/.config/rayneo/magcal.bin", home);
    return path;
}

void rayneo_magcal_identity(rayneo_magcal *cal)
{
    memset(cal, 0, sizeof *cal);
    cal->scale[0][0] = 1.0f;
    cal->scale[1][1] = 1.0f;
    cal->scale[2][2] = 1.0f;
}

/* binary format: 3 bias floats + 9 scale floats, little-endian */
int rayneo_magcal_load(rayneo_magcal *cal, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    float buf[12];
    if (fread(buf, sizeof(float), 12, f) != 12) { fclose(f); return -1; }
    fclose(f);
    for (int i = 0; i < 3; i++) cal->bias[i] = buf[i];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            cal->scale[r][c] = buf[3 + r*3 + c];
    return 0;
}

void rayneo_magcal_apply(const rayneo_magcal *cal,
                         const float in[3], float out[3])
{
    float tmp[3];
    for (int i = 0; i < 3; i++) tmp[i] = in[i] - cal->bias[i];
    for (int i = 0; i < 3; i++) {
        out[i] = 0.0f;
        for (int j = 0; j < 3; j++) out[i] += cal->scale[i][j] * tmp[j];
    }
}
