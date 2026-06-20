/* gltf_impl.cpp - single TU that compiles the vendored single-header libraries
 * the pet's glTF model path needs: cgltf (glTF 2.0 parsing) and stb_image
 * (PNG/JPEG decode for the model's textures). Kept in its own TU exactly like
 * stb_truetype_impl.cpp so the heavy headers compile once. */
#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#include "stb_image.h"
