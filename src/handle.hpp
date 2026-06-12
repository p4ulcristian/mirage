/* handle.hpp - tiny move-only RAII wrappers for the C resources mirage juggles.
 *
 * Each owns one handle and frees it on destruction, so a function that bails
 * mid-setup can't leak (or, worse, leave a half-built resource behind for a
 * later reuse-guard to hand back - the bug that once crashed a whole capture
 * session). Build locals, and on the success path `release()` them into the
 * long-lived screen_t (whose explicit *_finish still frees them, unchanged).
 *
 * Deliberately minimal: get()/release()/reset(), no copy, cheap move. Not a
 * unique_ptr<T,Deleter> because the deleters here are plain C calls and the
 * call sites read clearer with named types.
 */
#ifndef MIRAGE_HANDLE_HPP
#define MIRAGE_HANDLE_HPP

#include <unistd.h>
#include <gbm.h>
#include <wayland-client.h>
#include <GLES2/gl2.h>

/* namespace `own` = owning handles. (Not `mirage` - that name is taken by the
 * `struct mirage` app-state, and a namespace can't share it.) */
namespace own {

/* A POSIX file descriptor (the capture dmabuf fd). -1 = empty. */
class Fd {
    int fd_ = -1;
public:
    Fd() = default;
    explicit Fd(int fd) : fd_(fd) {}
    ~Fd() { if (fd_ >= 0) close(fd_); }
    Fd(Fd &&o) noexcept : fd_(o.fd_) { o.fd_ = -1; }
    Fd &operator=(Fd &&o) noexcept { reset(); fd_ = o.fd_; o.fd_ = -1; return *this; }
    Fd(const Fd &) = delete;
    Fd &operator=(const Fd &) = delete;
    int  get() const { return fd_; }
    bool valid() const { return fd_ >= 0; }
    int  release() { int f = fd_; fd_ = -1; return f; }
    void reset() { if (fd_ >= 0) close(fd_); fd_ = -1; }
};

/* A gbm buffer object (the capture target backing store). */
class GbmBo {
    struct gbm_bo *bo_ = nullptr;
public:
    GbmBo() = default;
    explicit GbmBo(struct gbm_bo *bo) : bo_(bo) {}
    ~GbmBo() { if (bo_) gbm_bo_destroy(bo_); }
    GbmBo(GbmBo &&o) noexcept : bo_(o.bo_) { o.bo_ = nullptr; }
    GbmBo &operator=(GbmBo &&o) noexcept { reset(); bo_ = o.bo_; o.bo_ = nullptr; return *this; }
    GbmBo(const GbmBo &) = delete;
    GbmBo &operator=(const GbmBo &) = delete;
    struct gbm_bo *get() const { return bo_; }
    explicit operator bool() const { return bo_ != nullptr; }
    struct gbm_bo *release() { struct gbm_bo *b = bo_; bo_ = nullptr; return b; }
    void reset() { if (bo_) gbm_bo_destroy(bo_); bo_ = nullptr; }
};

/* A wl_buffer wrapping the dmabuf for the compositor to copy into. */
class WlBuffer {
    struct wl_buffer *buf_ = nullptr;
public:
    WlBuffer() = default;
    explicit WlBuffer(struct wl_buffer *b) : buf_(b) {}
    ~WlBuffer() { if (buf_) wl_buffer_destroy(buf_); }
    WlBuffer(WlBuffer &&o) noexcept : buf_(o.buf_) { o.buf_ = nullptr; }
    WlBuffer &operator=(WlBuffer &&o) noexcept { reset(); buf_ = o.buf_; o.buf_ = nullptr; return *this; }
    WlBuffer(const WlBuffer &) = delete;
    WlBuffer &operator=(const WlBuffer &) = delete;
    struct wl_buffer *get() const { return buf_; }
    explicit operator bool() const { return buf_ != nullptr; }
    struct wl_buffer *release() { struct wl_buffer *b = buf_; buf_ = nullptr; return b; }
    void reset() { if (buf_) wl_buffer_destroy(buf_); buf_ = nullptr; }
};

/* ---- GL object handles ------------------------------------------------------
 * Each implicitly converts to its GLuint name, so existing call sites
 * (glUseProgram(prog), glBindBuffer(.., vbo), glBindTexture(.., tex), if(tex)...)
 * are unchanged; only creation (gen()/create()) and the move-assign-frees-old
 * pattern differ. These live in long-lived globals, so reset() them in
 * *_finish while the GL context is still current - never let a destructor run
 * after eglTerminate. */
class GlBuffer {
    GLuint id_ = 0;
public:
    GlBuffer() = default;
    ~GlBuffer() { if (id_) glDeleteBuffers(1, &id_); }
    GlBuffer(GlBuffer &&o) noexcept : id_(o.id_) { o.id_ = 0; }
    GlBuffer &operator=(GlBuffer &&o) noexcept { reset(); id_ = o.id_; o.id_ = 0; return *this; }
    GlBuffer(const GlBuffer &) = delete;
    GlBuffer &operator=(const GlBuffer &) = delete;
    operator GLuint() const { return id_; }
    GLuint *put() { reset(); return &id_; }     /* for glGenBuffers(1, x.put()) */
    void gen() { glGenBuffers(1, put()); }
    void reset() { if (id_) glDeleteBuffers(1, &id_); id_ = 0; }
};

class GlTexture {
    GLuint id_ = 0;
public:
    GlTexture() = default;
    ~GlTexture() { if (id_) glDeleteTextures(1, &id_); }
    GlTexture(GlTexture &&o) noexcept : id_(o.id_) { o.id_ = 0; }
    GlTexture &operator=(GlTexture &&o) noexcept { reset(); id_ = o.id_; o.id_ = 0; return *this; }
    GlTexture(const GlTexture &) = delete;
    GlTexture &operator=(const GlTexture &) = delete;
    operator GLuint() const { return id_; }
    GLuint *put() { reset(); return &id_; }
    void gen() { glGenTextures(1, put()); }
    void reset() { if (id_) glDeleteTextures(1, &id_); id_ = 0; }
};

class GlProgram {
    GLuint id_ = 0;
public:
    GlProgram() = default;
    ~GlProgram() { if (id_) glDeleteProgram(id_); }
    GlProgram(GlProgram &&o) noexcept : id_(o.id_) { o.id_ = 0; }
    GlProgram &operator=(GlProgram &&o) noexcept { reset(); id_ = o.id_; o.id_ = 0; return *this; }
    GlProgram(const GlProgram &) = delete;
    GlProgram &operator=(const GlProgram &) = delete;
    operator GLuint() const { return id_; }
    void create() { reset(); id_ = glCreateProgram(); }
    void reset() { if (id_) glDeleteProgram(id_); id_ = 0; }
};

} // namespace own

#endif /* MIRAGE_HANDLE_HPP */
