#!/usr/bin/env bash
# fetch.sh - download mirage's dome environments and convert them to the FLAT
# Radiance .hdr that load_hdri_rgb8() reads. The .exr/.hdr are git-ignored (large
# binaries), so run this once after cloning to populate the assets:
#
#     bash hdri/fetch.sh
#
#  - Space: NASA "Deep Star Maps 2020" (4K EXR) -> exr2hdr.py
#  - Forest/Mountain/Lake: CC0 night HDRIs from Poly Haven (4K .hdr, RLE) -> hdr2flat.py
#    chosen dark/dusk so they read on the additive optics (black = transparent).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

# --- Space (NASA, EXR) ---
# 8K (8192x4096): ~11 px/deg -> ~22 px/deg, matching the glasses better than 4K.
# The dome uploads as GL_RGB8 so bit depth is moot; resolution is the win. 16K also
# exists at this path but hits the Apple-GPU GL_MAX_TEXTURE_SIZE ceiling (16384).
URL="https://svs.gsfc.nasa.gov/vis/a000000/a004800/a004851/starmap_2020_8k.exr"
[ -f starmap_2020_8k.exr ] || { echo "downloading starmap"; curl -fL -o starmap_2020_8k.exr "$URL"; }
python3 exr2hdr.py starmap_2020_8k.exr starmap_2020_8k.hdr
echo "done -> hdri/starmap_2020_8k.hdr"

# --- Nature (Poly Haven, RLE .hdr) ---
# Cloudflare gates default curl UAs and intermittently 404s heavy GETs, so use a
# browser UA and retry. slug -> outfile must match MIRAGE_ENVS in config.cpp.
PH="https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/4k"
UA="Mozilla/5.0 (X11; Linux aarch64) AppleWebKit/537.36 Chrome/126 Safari/537.36"
fetch_ph() {  # slug  outname
    local slug="$1" out="$2" rle="${1}_rle.hdr"
    if [ ! -s "$rle" ]; then
        echo "downloading $slug"
        for i in 1 2 3 4 5 6; do
            code=$(curl -s -A "$UA" -o "$rle" -w "%{http_code}" "$PH/${slug}_4k.hdr") || true
            [ "$code" = "200" ] && [ "$(stat -c%s "$rle" 2>/dev/null || echo 0)" -gt 1000000 ] && break
            echo "  retry $i (http=$code)"; sleep 4
        done
    fi
    python3 hdr2flat.py "$rle" "$out"
    echo "done -> hdri/$out"
}
fetch_ph narrow_moonlit_road forest_night_4k.hdr     # Forest  (moonlit woods)
fetch_ph clarens_night_01    mountain_dusk_4k.hdr     # Mountain (moonlit hilltop + stars)
fetch_ph lakeside_night      lake_moonlit_4k.hdr      # Lake    (calm water + shore)

echo "all environments ready."
