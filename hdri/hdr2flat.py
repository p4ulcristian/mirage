#!/usr/bin/env python3
"""Radiance .hdr (new-style RLE OR already-flat) -> FLAT .hdr that mirage's
load_hdri_rgb8() reads (it rejects RLE). Poly Haven ships HDRIs RLE-compressed;
this flattens them with no quality loss (RGBE bytes are passed through verbatim).

Pure numpy + stdlib, so it runs anywhere (the Asahi box has no OpenEXR/ImageMagick).

    python3 hdr2flat.py in.hdr out.hdr
"""
import sys
import numpy as np

def read_radiance(path):
    d = open(path, "rb").read()
    p = 0
    def line():
        nonlocal p
        s = p
        while d[p] != 0x0a:
            p += 1
        out = d[s:p].decode("latin1"); p += 1
        return out
    assert line().startswith("#?"), "not a Radiance .hdr"
    while line() != "":            # header lines until the blank separator
        pass
    res = line().split()           # e.g. "-Y 2048 +X 4096"
    assert res[0] in ("-Y", "+Y") and res[2] in ("+X", "-X"), "bad resolution: " + " ".join(res)
    H, W = int(res[1]), int(res[3])

    out = np.empty((H, W, 4), np.uint8)
    for y in range(H):
        # new-style RLE scanline: 0x02 0x02 (W>>8) (W&0xff), then 4 RLE channels
        if (p + 4 <= len(d) and d[p] == 2 and d[p+1] == 2
                and ((d[p+2] << 8) | d[p+3]) == W and 8 <= W <= 0x7fff):
            p += 4
            for c in range(4):
                x = 0
                while x < W:
                    cnt = d[p]; p += 1
                    if cnt > 128:                      # run of (cnt-128) copies
                        n = cnt - 128
                        out[y, x:x+n, c] = d[p]; p += 1
                    else:                              # cnt literal bytes
                        n = cnt
                        out[y, x:x+n, c] = np.frombuffer(d, np.uint8, n, p); p += n
                    x += n
        else:                                          # already flat: W*4 raw RGBE
            out[y] = np.frombuffer(d, np.uint8, W*4, p).reshape(W, 4); p += W*4
    return W, H, out

if __name__ == "__main__":
    src, dst = sys.argv[1], sys.argv[2]
    W, H, rgbe = read_radiance(src)
    with open(dst, "wb") as f:
        f.write(b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n")
        f.write(("-Y %d +X %d\n" % (H, W)).encode())
        f.write(rgbe.tobytes())
    print("wrote %s  %dx%d  %d bytes (flat)" % (dst, W, H, H*W*4))
