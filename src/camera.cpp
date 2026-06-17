/* camera.cpp - V4L2 MJPEG capture + turbojpeg decode, on a background thread.
 * See camera.h. Triple-buffered: the capture thread writes one buffer, publishes it
 * as "ready", and the render thread claims it as "display" - so a frame the renderer
 * is uploading is never overwritten mid-flight, and acquire never copies or blocks
 * on decode. */
#include "camera.h"

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <cstring>
#include <cstdio>
#include <cerrno>

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <turbojpeg.h>

struct cam {
    int     fd = -1;
    int     w = 0, h = 0;
    struct Buf { void *start = nullptr; size_t len = 0; };
    Buf     vbuf[6];
    int     nbuf = 0;
    tjhandle tj = nullptr;

    std::thread       th;
    std::atomic<bool> run{false};

    std::mutex            mtx;          /* guards the triple-buffer indices + seq */
    std::vector<uint8_t>  rgb[3];
    int      idx_write = 0, idx_ready = -1, idx_disp = -1;
    uint64_t seq = 0;
};

static int xioctl(int fd, unsigned long req, void *arg) {
    int r;
    do { r = ioctl(fd, req, arg); } while (r < 0 && errno == EINTR);
    return r;
}

static void capture_loop(cam *c) {
    while (c->run.load(std::memory_order_relaxed)) {
        struct pollfd pfd{ c->fd, POLLIN, 0 };
        int pr = poll(&pfd, 1, 200);                 /* 200ms so stop() is responsive */
        if (pr <= 0) continue;                       /* timeout or signal: re-check run */
        if (!(pfd.revents & POLLIN)) continue;

        v4l2_buffer b{};
        b.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        if (xioctl(c->fd, VIDIOC_DQBUF, &b) < 0) {
            if (errno == EAGAIN) continue;
            break;                                   /* device went away */
        }

        uint8_t *dst = c->rgb[c->idx_write].data();
        bool ok = tjDecompress2(c->tj, (uint8_t *)c->vbuf[b.index].start, b.bytesused,
                                dst, c->w, 0, c->h, TJPF_RGB, TJFLAG_FASTDCT) == 0;
        if (ok) {
            std::lock_guard<std::mutex> lk(c->mtx);
            c->idx_ready = c->idx_write;
            c->seq++;
            for (int i = 0; i < 3; i++)              /* next write = neither ready nor displayed */
                if (i != c->idx_ready && i != c->idx_disp) { c->idx_write = i; break; }
        }
        xioctl(c->fd, VIDIOC_QBUF, &b);
    }
}

cam *cam_start(const char *dev, int w, int h) {
    cam *c = new cam();
    c->w = w; c->h = h;

    c->fd = open(dev, O_RDWR | O_NONBLOCK, 0);
    if (c->fd < 0) { std::fprintf(stderr, "camera: open %s failed (%s)\n", dev, strerror(errno)); delete c; return nullptr; }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = w;
    fmt.fmt.pix.height      = h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;
    if (xioctl(c->fd, VIDIOC_S_FMT, &fmt) < 0) {
        std::fprintf(stderr, "camera: S_FMT MJPEG %dx%d failed (%s)\n", w, h, strerror(errno));
        close(c->fd); delete c; return nullptr;
    }
    /* the driver may hand back a different size than asked - honour it */
    c->w = w = fmt.fmt.pix.width;
    c->h = h = fmt.fmt.pix.height;

    v4l2_requestbuffers req{};
    req.count  = 4;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(c->fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
        std::fprintf(stderr, "camera: REQBUFS failed (%s)\n", strerror(errno));
        close(c->fd); delete c; return nullptr;
    }
    c->nbuf = (int)req.count;
    for (int i = 0; i < c->nbuf; i++) {
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        if (xioctl(c->fd, VIDIOC_QUERYBUF, &b) < 0) { std::fprintf(stderr, "camera: QUERYBUF\n"); goto fail; }
        c->vbuf[i].len   = b.length;
        c->vbuf[i].start = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, c->fd, b.m.offset);
        if (c->vbuf[i].start == MAP_FAILED) { std::fprintf(stderr, "camera: mmap\n"); goto fail; }
        if (xioctl(c->fd, VIDIOC_QBUF, &b) < 0) { std::fprintf(stderr, "camera: QBUF\n"); goto fail; }
    }

    for (int i = 0; i < 3; i++) c->rgb[i].assign((size_t)w * h * 3, 0);
    c->tj = tjInitDecompress();
    if (!c->tj) { std::fprintf(stderr, "camera: tjInitDecompress\n"); goto fail; }

    {
        v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(c->fd, VIDIOC_STREAMON, &t) < 0) { std::fprintf(stderr, "camera: STREAMON (%s)\n", strerror(errno)); goto fail; }
    }

    c->run.store(true);
    c->th = std::thread(capture_loop, c);
    std::fprintf(stderr, "camera: streaming %s %dx%d MJPEG\n", dev, w, h);
    return c;

fail:
    if (c->tj) tjDestroy(c->tj);
    for (int i = 0; i < c->nbuf; i++) if (c->vbuf[i].start && c->vbuf[i].start != MAP_FAILED) munmap(c->vbuf[i].start, c->vbuf[i].len);
    if (c->fd >= 0) close(c->fd);
    delete c;
    return nullptr;
}

void cam_stop(cam *c) {
    if (!c) return;
    c->run.store(false);
    if (c->th.joinable()) c->th.join();
    v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(c->fd, VIDIOC_STREAMOFF, &t);
    for (int i = 0; i < c->nbuf; i++) if (c->vbuf[i].start && c->vbuf[i].start != MAP_FAILED) munmap(c->vbuf[i].start, c->vbuf[i].len);
    if (c->tj) tjDestroy(c->tj);
    if (c->fd >= 0) close(c->fd);
    std::fprintf(stderr, "camera: stopped\n");
    delete c;
}

bool cam_acquire(cam *c, const uint8_t **rgb, int *w, int *h, uint64_t *seq) {
    if (!c) return false;
    std::lock_guard<std::mutex> lk(c->mtx);
    if (c->idx_ready < 0 || c->seq == *seq) return false;
    c->idx_disp = c->idx_ready;
    *rgb = c->rgb[c->idx_disp].data();
    *w = c->w; *h = c->h; *seq = c->seq;
    return true;
}
