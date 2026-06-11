/*
 * intercept_carina.c - Intercept carina_vio_init to dump the config string
 *
 * The trick: We use DYLD_INTERPOSE to replace the function. On macOS, this
 * requires the binary to be unsigned or have get-task-allow entitlement.
 * SpaceWalker is signed, so we try a different approach: just load before
 * libcarina_vio and define our own symbol.
 *
 * Actually, the simplest way is to use lldb. But here's a helper that
 * reads /proc-like info from the process.
 *
 * Build: clang -shared -o intercept_carina.dylib intercept_carina.c
 * Use: May not work due to code signing. Try lldb instead.
 */

#include <stdio.h>
#include <string.h>

/* Just a placeholder - the real solution is lldb or dtrace */
__attribute__((constructor))
static void on_load(void) {
    fprintf(stderr, "[intercept_carina] loaded\n");
}
