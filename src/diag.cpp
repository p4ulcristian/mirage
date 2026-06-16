#include "diag.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <pthread.h>
#include <time.h>

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static FILE   *g_f   = NULL;
static int     g_on  = -1;      /* -1 = undecided, 0 = off, 1 = on */
static double  g_t0  = -1.0;

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

int diag_enabled(void) {
    if (g_on < 0) {
        const char *e = getenv("MIRAGE_DIAG");
        g_on = (e && e[0] == '0') ? 0 : 1;     /* default ON */
    }
    return g_on;
}

void diag_logf(const char *tag, const char *fmt, ...) {
    if (!diag_enabled()) return;
    pthread_mutex_lock(&g_lock);
    if (!g_f) {
        const char *p = getenv("MIRAGE_DIAG_LOG");
        g_f = fopen((p && *p) ? p : "/tmp/mirage-diag.log", "a");
        if (g_f) {
            g_t0 = now_ms();
            fprintf(g_f, "\n==== mirage diag session start ====\n");
        } else {
            g_on = 0;                          /* couldn't open: give up quietly */
        }
    }
    if (g_f) {
        fprintf(g_f, "%9.1f %-4s ", now_ms() - g_t0, tag);
        va_list ap; va_start(ap, fmt);
        vfprintf(g_f, fmt, ap);
        va_end(ap);
        fputc('\n', g_f);
        fflush(g_f);
    }
    pthread_mutex_unlock(&g_lock);
}
