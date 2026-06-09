#pragma once

typedef struct {
    float bias[3];
    float scale[3][3];
} rayneo_magcal;

const char *rayneo_magcal_default_path(void);
int  rayneo_magcal_load(rayneo_magcal *cal, const char *path);
void rayneo_magcal_identity(rayneo_magcal *cal);
void rayneo_magcal_apply(const rayneo_magcal *cal,
                         const float in[3], float out[3]);
