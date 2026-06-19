/* Lone translation unit that compiles the vendored Clay layout library.
 * Clay is a single-header lib (src/clay.h); exactly one TU must define
 * CLAY_IMPLEMENTATION. Mirrors stb_truetype_impl.cpp. Clay supports C++
 * compilation, so it rides the project's gnu++23 like every other source. */
#define CLAY_IMPLEMENTATION
#include "clay.h"
