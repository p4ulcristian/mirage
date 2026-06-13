/* mirage-cal: pre-flight calibration. Verifies the tracker is streaming, lets you
 * recenter and dial in the head-tracking feel + FOV while watching a live readout,
 * then writes profile.toml that mirage loads over its defaults. Terminal UI - none
 * of this needs the glasses, so you sort out a dead bridge or a flipped axis on the
 * laptop BEFORE going head-tracked.
 *
 *   mirage-cal                 # reads/writes the default profile.toml
 *   mirage-cal --port 5005     # override the tracker UDP port for this session
 */
#include "mirage.h"
#include "pose.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <ctime>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <print>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

/* ---- raw terminal: read single keys with no Enter, no echo, non-blocking ---- */
static struct termios g_saved_tio;
static bool g_raw = false;
static void raw_restore(void) {
    if (g_raw) { tcsetattr(STDIN_FILENO, TCSANOW, &g_saved_tio); g_raw = false; }
    std::print("\x1b[?25h");   /* show cursor */
    fflush(stdout);
}
static void raw_enable(void) {
    if (tcgetattr(STDIN_FILENO, &g_saved_tio) != 0) return;
    struct termios t = g_saved_tio;
    t.c_lflag &= ~(ICANON | ECHO);
    t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;     /* read returns immediately */
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    g_raw = true;
    atexit(raw_restore);
    std::print("\x1b[?25l");   /* hide cursor */
}
static int read_key(void) { char c; return (read(STDIN_FILENO, &c, 1) == 1) ? (unsigned char)c : -1; }

/* quaternion -> yaw/pitch/roll degrees, matching mirage-posedump. */
static void quat_to_ypr_deg(quat q, float *yaw, float *pitch, float *roll) {
    float sinp = 2.0f*(q.w*q.x - q.y*q.z);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp) * 180.0f/(float)M_PI;
    *yaw   = atan2f(2.0f*(q.w*q.y + q.x*q.z), 1.0f - 2.0f*(q.x*q.x + q.y*q.y)) * 180.0f/(float)M_PI;
    *roll  = atan2f(2.0f*(q.w*q.z + q.x*q.y), 1.0f - 2.0f*(q.x*q.x + q.z*q.z)) * 180.0f/(float)M_PI;
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static double now_s(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
                            return ts.tv_sec + ts.tv_nsec/1e9; }

/* a labelled +/- direction arrow for the live axis check */
static const char *arrow(float v, const char *neg, const char *pos) {
    return v < -3.0f ? neg : (v > 3.0f ? pos : "  ·  ");
}

int main(int argc, char **argv) {
    std::string path = profile_default_path();
    mirage_config c; mirage_config_defaults(&c);
    int loaded = profile_load(path.c_str(), &c);   /* start from existing profile if any */

    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--port") && i+1 < argc) c.pose_port = atoi(argv[++i]);

    pose_config pc = { .backend = POSE_OPENTRACK_UDP, .udp_port = c.pose_port,
                       .socket_path = nullptr, .smoothing = c.pose_smoothing,
                       .use_oneeuro = c.pose_oneeuro, .oe_mincutoff = c.pose_mincutoff,
                       .oe_beta = c.pose_beta, .oe_dcutoff = 1.0f,
                       .sign_yaw = 1.0f, .sign_pitch = 1.0f, .sign_roll = 1.0f,
                       .drift_comp_tau = c.pose_drift_tau };
    if (pose_start(&pc) != 0) { std::print(stderr, "mirage-cal: pose backend failed to start\n"); return 1; }

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    raw_enable();

    double last_rate_t = now_s(); float hz = 0.0f; const char *msg = loaded > 0 ? "loaded existing profile" : "no profile yet - tuning from defaults";

    while (!g_stop) {
        /* ---- input ---- */
        int k;
        while ((k = read_key()) != -1) {
            switch (k) {
            case 'r': pose_recenter(); msg = "recentered: this is now 'straight ahead'"; break;
            case 'f': c.fov_deg        = clampf(c.fov_deg - 0.5f, 30, 120); break;
            case 'F': c.fov_deg        = clampf(c.fov_deg + 0.5f, 30, 120); break;
            case 'y': c.yaw_gain       = clampf(c.yaw_gain - 0.25f, 0.5f, 15); break;
            case 'Y': c.yaw_gain       = clampf(c.yaw_gain + 0.25f, 0.5f, 15); break;
            case 'p': c.pitch_gain     = clampf(c.pitch_gain - 0.25f, 0.5f, 15); break;
            case 'P': c.pitch_gain     = clampf(c.pitch_gain + 0.25f, 0.5f, 15); break;
            case 'd': c.pose_drift_tau = clampf(c.pose_drift_tau - 0.25f, 0, 5); break;
            case 'D': c.pose_drift_tau = clampf(c.pose_drift_tau + 0.25f, 0, 5); break;
            case 'm': c.pose_mincutoff = clampf(c.pose_mincutoff - 0.05f, 0.05f, 5); break;
            case 'M': c.pose_mincutoff = clampf(c.pose_mincutoff + 0.05f, 0.05f, 5); break;
            case 's':
                msg = profile_save(path.c_str(), &c) ? "SAVED profile.toml" : "SAVE FAILED";
                break;
            case 'q': case 3: g_stop = 1; break;
            default: break;
            }
        }

        /* ---- sample rate (Hz) over the redraw interval ---- */
        double t = now_s(); double dt = t - last_rate_t;
        if (dt >= 0.5) { hz = pose_take_sample_count() / (float)dt; last_rate_t = t; }

        /* ---- draw ---- */
        quat q = pose_latest(); float y, p, r; quat_to_ypr_deg(q, &y, &p, &r);
        bool sig = pose_has_signal();
        uint32_t age = pose_age_ms();

        std::print("\x1b[2J\x1b[H");   /* clear + home */
        std::print("  mirage-cal — head-tracking pre-flight       profile: {}\n", path);
        std::print("  ───────────────────────────────────────────────────────────────\n");
        std::print("  tracker : {}   {:5.1f} Hz   age {:>4} ms   port {}\n",
                   sig ? "\x1b[32mSTREAMING\x1b[0m" : "\x1b[31mNO SIGNAL — is the bridge running?\x1b[0m",
                   hz, age == UINT32_MAX ? 0u : age, c.pose_port);
        std::print("\n");
        std::print("  AXIS CHECK — move your head, the arrow should follow:\n");
        std::print("     yaw  (turn L/R) : {: 7.2f}°   {}\n", y, arrow(y, "\x1b[36m LEFT\x1b[0m", "\x1b[36mRIGHT\x1b[0m"));
        std::print("     pitch(nod U/D)  : {: 7.2f}°   {}\n", p, arrow(p, "\x1b[36m DOWN\x1b[0m", "\x1b[36m  UP \x1b[0m"));
        std::print("     roll (tilt)     : {: 7.2f}°\n", r);
        std::print("\n");
        std::print("  TUNING (live):\n");
        std::print("     FOV            f/F : {:6.1f}°\n", c.fov_deg);
        std::print("     yaw gain       y/Y : {:6.2f}\n", c.yaw_gain);
        std::print("     pitch gain     p/P : {:6.2f}\n", c.pitch_gain);
        std::print("     drift cancel   d/D : {:6.2f} s\n", c.pose_drift_tau);
        std::print("     steadiness     m/M : {:6.2f} Hz (lower = steadier)\n", c.pose_mincutoff);
        std::print("\n");
        std::print("  [r] recenter   [s] save   [q] quit\n");
        std::print("  ───────────────────────────────────────────────────────────────\n");
        std::print("  {}\n", msg);
        fflush(stdout);

        usleep(100000);   /* 10 Hz redraw */
    }

    raw_restore();
    pose_stop();
    std::print("\nmirage-cal: done. Launch mirage to use the calibration.\n");
    return 0;
}
