/* math3d.h - minimal vec/quat/mat4 math for mirage (no glm dependency).
 *
 * Conventions:
 *   - Right-handed coordinate system. -Z is "forward" (into the scene),
 *     +X right, +Y up. This matches OpenGL clip space expectations.
 *   - mat4 is column-major (m[col*4 + row]), directly uploadable to GLES.
 *   - quat is {w,x,y,z}, unit-length for rotations.
 */
#ifndef MIRAGE_MATH3D_H
#define MIRAGE_MATH3D_H

#include <math.h>

typedef struct { float x, y, z; } vec3;
typedef struct { float w, x, y, z; } quat;
typedef struct { float m[16]; } mat4;

static inline vec3 v3(float x, float y, float z) { return (vec3){x, y, z}; }
static inline vec3 v3_add(vec3 a, vec3 b) { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline vec3 v3_sub(vec3 a, vec3 b) { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline vec3 v3_scale(vec3 a, float s) { return v3(a.x*s, a.y*s, a.z*s); }
static inline float v3_dot(vec3 a, vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline vec3 v3_cross(vec3 a, vec3 b) {
    return v3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
static inline float v3_len(vec3 a) { return sqrtf(v3_dot(a, a)); }
static inline vec3 v3_norm(vec3 a) {
    float l = v3_len(a);
    return l > 1e-8f ? v3_scale(a, 1.0f/l) : a;
}

/* ---- quaternions ---- */
static inline quat q_identity(void) { return (quat){1, 0, 0, 0}; }

static inline quat q_norm(quat q) {
    float l = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (l < 1e-8f) return q_identity();
    float inv = 1.0f/l;
    return (quat){q.w*inv, q.x*inv, q.y*inv, q.z*inv};
}

static inline quat q_conj(quat q) { return (quat){q.w, -q.x, -q.y, -q.z}; }

static inline quat q_mul(quat a, quat b) {
    return (quat){
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

/* Rotate a vector by a unit quaternion. */
static inline vec3 q_rotate(quat q, vec3 v) {
    vec3 u = v3(q.x, q.y, q.z);
    float s = q.w;
    vec3 t = v3_scale(v3_cross(u, v), 2.0f);
    return v3_add(v3_add(v, v3_scale(t, s)), v3_cross(u, t));
}

/* Spherical-ish linear interpolation (nlerp; cheap, stable for small steps). */
static inline quat q_nlerp(quat a, quat b, float t) {
    /* take shortest path */
    float d = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    if (d < 0.0f) { b.w=-b.w; b.x=-b.x; b.y=-b.y; b.z=-b.z; }
    quat r = {
        a.w + (b.w-a.w)*t, a.x + (b.x-a.x)*t,
        a.y + (b.y-a.y)*t, a.z + (b.z-a.z)*t
    };
    return q_norm(r);
}

/* Build a quaternion from intrinsic yaw(Y)->pitch(X)->roll(Z) euler in radians.
 * Matches OpenTrack's yaw/pitch/roll semantics. */
static inline quat q_from_euler_ypr(float yaw, float pitch, float roll) {
    float cy = cosf(yaw*0.5f),   sy = sinf(yaw*0.5f);
    float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
    float cr = cosf(roll*0.5f),  sr = sinf(roll*0.5f);
    quat qy = {cy, 0, sy, 0};   /* yaw about +Y */
    quat qx = {cp, sp, 0, 0};   /* pitch about +X */
    quat qz = {cr, 0, 0, sr};   /* roll about +Z */
    return q_norm(q_mul(q_mul(qy, qx), qz));
}

/* Inverse of q_from_euler_ypr: decompose a unit quaternion back into the same
 * intrinsic yaw(Y)->pitch(X)->roll(Z) angles (radians). Derived from the rows
 * of R = Ry(yaw)*Rx(pitch)*Rz(roll); stable except at pitch = +-90 deg (gimbal
 * lock), which a head looking at a desktop never reaches. */
static inline void q_to_euler_ypr(quat q, float *yaw, float *pitch, float *roll) {
    float w=q.w, x=q.x, y=q.y, z=q.z;
    float m12 = 2.0f*(y*z - w*x);          /* R[1][2] = -sin(pitch) */
    float sp  = -m12; if (sp < -1.0f) sp = -1.0f; if (sp > 1.0f) sp = 1.0f;
    *pitch = asinf(sp);
    *yaw   = atan2f(2.0f*(x*z + w*y), 1.0f - 2.0f*(x*x + y*y));  /* R02, R22 */
    *roll  = atan2f(2.0f*(x*y + w*z), 1.0f - 2.0f*(x*x + z*z));  /* R10, R11 */
}

/* ---- mat4 (column-major) ---- */
static inline mat4 m4_identity(void) {
    mat4 r = {{0}};
    r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
    return r;
}

static inline mat4 m4_mul(mat4 a, mat4 b) {
    mat4 r;
    for (int c = 0; c < 4; c++)
        for (int row = 0; row < 4; row++) {
            float s = 0;
            for (int k = 0; k < 4; k++) s += a.m[k*4+row] * b.m[c*4+k];
            r.m[c*4+row] = s;
        }
    return r;
}

static inline mat4 m4_translate(vec3 t) {
    mat4 r = m4_identity();
    r.m[12] = t.x; r.m[13] = t.y; r.m[14] = t.z;
    return r;
}

static inline mat4 m4_scale(vec3 s) {
    mat4 r = m4_identity();
    r.m[0] = s.x; r.m[5] = s.y; r.m[10] = s.z;
    return r;
}

/* Rotation matrix from unit quaternion. */
static inline mat4 m4_from_quat(quat q) {
    float x=q.x, y=q.y, z=q.z, w=q.w;
    mat4 r = m4_identity();
    r.m[0]  = 1 - 2*(y*y + z*z);
    r.m[1]  = 2*(x*y + z*w);
    r.m[2]  = 2*(x*z - y*w);
    r.m[4]  = 2*(x*y - z*w);
    r.m[5]  = 1 - 2*(x*x + z*z);
    r.m[6]  = 2*(y*z + x*w);
    r.m[8]  = 2*(x*z + y*w);
    r.m[9]  = 2*(y*z - x*w);
    r.m[10] = 1 - 2*(x*x + y*y);
    return r;
}

/* Orthographic projection (pixel space): x in [l,r], y in [b,t]. */
static inline mat4 m4_ortho(float l, float r, float b, float t, float n, float f) {
    mat4 m = m4_identity();
    m.m[0]  = 2.0f/(r-l);
    m.m[5]  = 2.0f/(t-b);
    m.m[10] = -2.0f/(f-n);
    m.m[12] = -(r+l)/(r-l);
    m.m[13] = -(t+b)/(t-b);
    m.m[14] = -(f+n)/(f-n);
    return m;
}

/* Perspective projection. fovy in radians. */
static inline mat4 m4_perspective(float fovy, float aspect, float znear, float zfar) {
    float f = 1.0f / tanf(fovy * 0.5f);
    mat4 r = {{0}};
    r.m[0]  = f / aspect;
    r.m[5]  = f;
    r.m[10] = (zfar + znear) / (znear - zfar);
    r.m[11] = -1.0f;
    r.m[14] = (2.0f * zfar * znear) / (znear - zfar);
    return r;
}

#endif /* MIRAGE_MATH3D_H */
