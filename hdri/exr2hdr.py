#!/usr/bin/env python3
"""Minimal OpenEXR (scanline, ZIP/ZIPS/none) -> Radiance .hdr converter.

Pure numpy + zlib (stdlib), so it runs anywhere without OpenEXR/freeimage.
Handles the HALF/FLOAT RGB scanline EXRs that NASA's star maps ship as.

    python3 exr2hdr.py in.exr out.hdr
"""
import sys, struct, zlib
import numpy as np

def read_exr(path):
    d = open(path, "rb").read()
    assert d[:4] == b"\x76\x2f\x31\x01", "not an EXR"
    p = 8  # magic(4) + version(4)

    def rstr():
        nonlocal p
        s = p
        while d[p] != 0:
            p += 1
        out = d[s:p]; p += 1
        return out.decode("latin1")

    attrs = {}
    while True:
        name = rstr()
        if name == "":
            break
        atype = rstr()
        size = struct.unpack_from("<i", d, p)[0]; p += 4
        val = d[p:p+size]; p += size
        attrs[name] = (atype, val)

    # channels: list of (name, pixtype) in file (alphabetical) order
    chans = []
    cb = attrs["channels"][1]; q = 0
    while cb[q] != 0:
        s = q
        while cb[q] != 0:
            q += 1
        cname = cb[s:q].decode("latin1"); q += 1
        pixtype = struct.unpack_from("<i", cb, q)[0]  # 0 UINT,1 HALF,2 FLOAT
        q += 16  # pixtype(4) + pLinear/reserved(4) + xSamp(4) + ySamp(4)
        chans.append((cname, pixtype))

    comp = attrs["compression"][1][0]                  # 0 none,1 RLE,2 ZIPS,3 ZIP
    xmin, ymin, xmax, ymax = struct.unpack_from("<iiii", attrs["dataWindow"][1], 0)
    W, H = xmax - xmin + 1, ymax - ymin + 1
    print("channels:", chans, "compression:", comp, "size:", W, "x", H)

    TYPESZ = {0: 4, 1: 2, 2: 4}
    TYPENP = {0: "<u4", 1: "<f2", 2: "<f4"}
    rowbytes = sum(W * TYPESZ[t] for _, t in chans)
    lpb = 16 if comp == 3 else 1                        # lines per block
    nblocks = (H + lpb - 1) // lpb

    p += nblocks * 8                                   # skip scanline offset table

    img = {c: np.zeros((H, W), np.float32) for c, _ in chans}

    def unzip(raw, usize):
        if len(raw) >= usize:                          # stored uncompressed
            tmp = np.frombuffer(raw[:usize], np.uint8).copy()
        else:
            tmp = np.frombuffer(zlib.decompress(raw), np.uint8).copy()
        a = tmp.astype(np.int64); a[1:] -= 128         # reverse delta predictor
        pred = (np.cumsum(a) & 0xff).astype(np.uint8)
        out = np.empty(usize, np.uint8)                # reverse byte interleave
        half = (usize + 1) // 2
        out[0::2] = pred[:half]; out[1::2] = pred[half:]
        return out

    for _ in range(nblocks):
        y = struct.unpack_from("<i", d, p)[0]; p += 4
        dsize = struct.unpack_from("<i", d, p)[0]; p += 4
        raw = d[p:p+dsize]; p += dsize
        lines = min(lpb, ymax - y + 1)
        buf = raw if comp in (0,) else unzip(raw, lines * rowbytes)
        buf = np.frombuffer(bytes(buf), np.uint8)
        off = 0
        for s in range(lines):
            for cname, t in chans:
                nb = W * TYPESZ[t]
                row = np.frombuffer(buf[off:off+nb].tobytes(), TYPENP[t]).astype(np.float32)
                img[cname][y - ymin + s] = row
                off += nb
    return W, H, img

def write_hdr(path, rgb):
    H, W, _ = rgb.shape
    rgb = np.nan_to_num(rgb, nan=0.0, posinf=0.0, neginf=0.0)
    rgb = np.clip(rgb, 0.0, None)
    m = rgb.max(axis=2)
    mant, expo = np.frexp(np.maximum(m, 0.0))
    nz = m > 1e-32
    scale = np.where(nz, mant * 256.0 / np.maximum(m, 1e-32), 0.0)[..., None]
    rgbe = np.zeros((H, W, 4), np.uint8)
    rgbe[..., :3] = np.clip(rgb * scale, 0, 255).astype(np.uint8)
    rgbe[..., 3] = np.where(nz, np.clip(expo + 128, 0, 255), 0).astype(np.uint8)
    with open(path, "wb") as f:
        f.write(b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n")
        f.write(("-Y %d +X %d\n" % (H, W)).encode())
        f.write(rgbe.tobytes())

if __name__ == "__main__":
    src, dst = sys.argv[1], sys.argv[2]
    W, H, img = read_exr(src)
    rgb = np.stack([img.get(c, np.zeros((H, W), np.float32)) for c in ("R", "G", "B")], axis=2)
    print("luminance: min %.4g max %.4g mean %.4g" % (rgb.min(), rgb.max(), rgb.mean()))
    lm = rgb.max(axis=2)
    print("frac<0.01 %.3f  frac<0.1 %.3f  frac>1 %.4f" %
          ((lm < 0.01).mean(), (lm < 0.1).mean(), (lm > 1).mean()))
    write_hdr(dst, rgb)
    print("wrote", dst)
