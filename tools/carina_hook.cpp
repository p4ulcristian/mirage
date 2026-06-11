/*
 * carina_hook.cpp - Intercept carina_vio_init to capture the config YAML
 *
 * Build:
 *   clang++ -dynamiclib -o carina_hook.dylib carina_hook.cpp \
 *     -std=c++17 -framework Foundation
 *
 * Use:
 *   1. Quit SpaceWalker
 *   2. DYLD_INSERT_LIBRARIES=./carina_hook.dylib /Applications/SpaceWalker.app/Contents/MacOS/SpaceWalker
 *   3. Check /tmp/carina_config.yaml
 */

#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <string>

// The real carina_vio_init signature (C++ mangled, but we use extern "C" wrapper)
typedef int (*carina_vio_init_t)(const std::string&, const std::string&);
static carina_vio_init_t real_init = nullptr;

extern "C" int carina_vio_init(const std::string& config, const std::string& fusion) {
    // Log the config to a file
    FILE *f = fopen("/tmp/carina_config.yaml", "wb");
    if (f) {
        fprintf(f, "# ===== CONFIG (len=%zu) =====\n", config.size());
        fwrite(config.data(), 1, config.size(), f);
        fprintf(f, "\n\n# ===== FUSION (len=%zu) =====\n", fusion.size());
        fwrite(fusion.data(), 1, fusion.size(), f);
        fclose(f);
        fprintf(stderr, "[carina_hook] Wrote config to /tmp/carina_config.yaml\n");
    }

    // Also print to stderr for immediate visibility
    fprintf(stderr, "[carina_hook] carina_vio_init called:\n");
    fprintf(stderr, "--- CONFIG (%zu bytes) ---\n%s\n", config.size(), config.c_str());
    fprintf(stderr, "--- FUSION (%zu bytes) ---\n%s\n", fusion.size(), fusion.c_str());

    // Find and call the real function
    if (!real_init) {
        void *lib = dlopen("/Applications/SpaceWalker.app/Contents/Frameworks/libcarina_vio.dylib", RTLD_NOW);
        if (lib) {
            // The function is C++ mangled - need the actual symbol name
            // _carina_vio_init is the extern "C" wrapper
            real_init = (carina_vio_init_t)dlsym(lib, "_carina_vio_init");
            if (!real_init) {
                // Try C++ mangled name (varies by compiler)
                real_init = (carina_vio_init_t)dlsym(lib, "_Z15carina_vio_initRKNSt3__112basic_stringIcNS_11char_traitsIcEENS_9allocatorIcEEEES7_");
            }
        }
    }

    if (real_init) {
        return real_init(config, fusion);
    }

    fprintf(stderr, "[carina_hook] ERROR: could not find real carina_vio_init\n");
    return -1;
}

// Constructor to log when we're loaded
__attribute__((constructor))
static void on_load() {
    fprintf(stderr, "[carina_hook] Loaded! Will intercept carina_vio_init calls.\n");
}
