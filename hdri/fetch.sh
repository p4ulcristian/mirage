#!/usr/bin/env bash
# fetch.sh - download the NASA "Deep Star Maps 2020" (4K) and convert it to the
# flat Radiance .hdr that mirage's dome loads. The .exr/.hdr are git-ignored
# (large binaries), so run this once after cloning to populate the asset.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

URL="https://svs.gsfc.nasa.gov/vis/a000000/a004800/a004851/starmap_2020_4k.exr"
[ -f starmap_2020_4k.exr ] || { echo "downloading $URL"; curl -fL -o starmap_2020_4k.exr "$URL"; }
python3 exr2hdr.py starmap_2020_4k.exr starmap_2020_4k.hdr
echo "done -> hdri/starmap_2020_4k.hdr"
