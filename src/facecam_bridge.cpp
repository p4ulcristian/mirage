/* facecam_bridge.cpp - webcam head-position tracker for mirage's 6DoF.
 *
 * The RayNeo IMU gives mirage head ORIENTATION (3DoF) but cannot sense head
 * POSITION - an accelerometer double-integrates into metres of drift in seconds.
 * This bridge supplies the missing 3 translational DoF the cheap way: it watches
 * you through the laptop webcam, finds your head, and reports where it is. mirage
 * fuses it (rotation from the IMU, position from here) for real lean/slide parallax
 * on the world-fixed wall - lean toward the wall and it comes closer; slide sideways
 * and you see around the near windows.
 *
 * No marker, no sticker: a face detector locates your head. We only need POSITION
 * (the IMU still owns rotation), and position survives the dark AR lenses fine - the
 * detector just needs to see your head, not your eyes.
 *
 *   webcam --> face detect --> {cx,cy, apparent-size} --> metric (x,y,distance)
 *          --> unix-dgram JSON {"pos":[x,y,z]} --> mirage pose layer
 *
 * Detector: YuNet (assets/face_detection_yunet_2023mar.onnx) if present - robust with
 * glasses and gives eye landmarks for a steadier depth estimate; otherwise it falls
 * back to OpenCV's bundled Haar cascade (works, but jitterier).
 *
 * Frame sent to mirage (mirage world axes, metres):
 *   x  +right   y  +up   z = camera->head distance (smaller = leaned in)
 * mirage subtracts a rest reference and negates z, so leaning in pushes the eye
 * forward. Sign/scale of the final effect is tuned live in mirage's config
 * (facecam_lateral_gain / facecam_depth_gain) - no rebuild here to flip an axis.
 *
 * Build:  make facecam     Run:  ./facecam-bridge   (start it alongside mirage)
 */
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <ctime>
#include <csignal>
#include <deque>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

struct Opts {
    std::string sock  = "/tmp/mirage-facecam.sock";
    std::string model = "assets/face_detection_yunet_2023mar.onnx";
    int   camera = 0;
    int   width  = 640;
    int   height = 480;
    float fov_h  = 55.0f;   /* horizontal field of view of the webcam, degrees */
    bool  mirror = false;   /* set if the cam delivers a mirrored (selfie) image */
    bool  verbose = false;
    int   threads = 2;      /* OpenCV worker threads (default 8 thrashes a busy box) */
    int   fps_cap = 30;     /* cap the processing rate; head position is slow */
};

static void usage(const char *p) {
    std::fprintf(stderr,
        "usage: %s [--socket PATH] [--camera N] [--model PATH] [--fov DEG]\n"
        "          [--size WxH] [--mirror] [--verbose]\n"
        "  Tracks head position via the webcam and streams it to mirage.\n"
        "  --mirror   flip left/right (use if leaning right moves the wall left)\n", p);
}

static bool parse_args(int argc, char **argv, Opts &o) {
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&](const char *name) -> const char * {
            if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", name); return nullptr; }
            return argv[++i];
        };
        if (a == "--socket")       { auto v = next("--socket"); if (!v) return false; o.sock = v; }
        else if (a == "--model")   { auto v = next("--model");  if (!v) return false; o.model = v; }
        else if (a == "--camera")  { auto v = next("--camera"); if (!v) return false; o.camera = atoi(v); }
        else if (a == "--fov")     { auto v = next("--fov");    if (!v) return false; o.fov_h = atof(v); }
        else if (a == "--size")    { auto v = next("--size");   if (!v) return false;
                                     sscanf(v, "%dx%d", &o.width, &o.height); }
        else if (a == "--mirror")  { o.mirror = true; }
        else if (a == "--verbose" || a == "-v") { o.verbose = true; }
        else if (a == "--help" || a == "-h") { usage(argv[0]); return false; }
        else { std::fprintf(stderr, "unknown arg: %s\n", a.c_str()); usage(argv[0]); return false; }
    }
    return true;
}

/* A single head observation in image space: centre pixel + a known-real-size span
 * (eye separation, or face width on the Haar path) used to recover metric depth. */
struct Head { float cx, cy, span_px, span_m; bool ok; };

static double now_s(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char **argv) {
    Opts o;
    if (!parse_args(argc, argv, o)) return 1;
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    /* Cap OpenCV's worker threads. YuNet barely parallelises at this size, but the
     * default (one per core) spawns 8 threads that thrash and burn ~130% CPU for ~10
     * fps on a busy machine. 2 is plenty and frees cores for mirage. */
    cv::setNumThreads(o.threads);

    /* --- camera --- */
    cv::VideoCapture cap(o.camera, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "facecam: cannot open camera %d\n", o.camera);
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  o.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, o.height);
    cap.set(cv::CAP_PROP_FPS, o.fps_cap);   /* head position is slow; no need for 100+ fps */
    int W = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int H = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    if (W <= 0 || H <= 0) { W = o.width; H = o.height; }

    /* pinhole focal length in pixels from the horizontal FOV; metric position needs
     * it. Rough is fine - absolute scale just feeds a tunable gain in mirage. */
    float f_px = (W * 0.5f) / std::tan(o.fov_h * 0.5f * (float)CV_PI / 180.0f);

    /* --- detector: YuNet if the model is there, else bundled Haar --- */
    cv::Ptr<cv::FaceDetectorYN> yunet;
    cv::CascadeClassifier haar;
    bool use_yunet = false;
    if (access(o.model.c_str(), R_OK) == 0) {
        try {
            yunet = cv::FaceDetectorYN::create(o.model, "", cv::Size(W, H), 0.7f, 0.3f, 5000);
            use_yunet = true;
        } catch (const cv::Exception &e) {
            std::fprintf(stderr, "facecam: YuNet load failed (%s), trying Haar\n", e.what());
        }
    }
    if (!use_yunet) {
        const char *hc = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml";
        if (!haar.load(hc)) {
            std::fprintf(stderr, "facecam: no YuNet model (%s) and Haar load failed (%s)\n",
                         o.model.c_str(), hc);
            return 1;
        }
    }
    std::fprintf(stderr, "facecam: %s detector, %dx%d, f=%.0fpx, fov=%.0f deg -> %s\n",
                 use_yunet ? "YuNet" : "Haar", W, H, f_px, o.fov_h, o.sock.c_str());

    /* --- outbound unix-dgram socket to mirage --- */
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) { perror("facecam: socket"); return 1; }
    struct sockaddr_un dst = {};
    dst.sun_family = AF_UNIX;
    std::strncpy(dst.sun_path, o.sock.c_str(), sizeof dst.sun_path - 1);

    cv::Mat frame, gray;
    double t_log = now_s(); long frames = 0; bool warned_send = false;
    /* outlier rejection: a misdetection (wrong face scale, spurious box) produces a
     * position that teleports from the last good one. At 100+ fps a real head move is
     * tiny per frame, so anything large is a glitch - drop it. After a few consecutive
     * rejects we accept anyway, so a genuine re-lock (you left + returned) recovers. */
    float pwx = 0, pwy = 0, pZ = 0; bool have_prev = false; int reject_streak = 0;
    /* median window on the raw measurement (centre + apparent size) BEFORE deriving
     * metric position. A median outvotes the occasional wrong-scale detection frame
     * outright - far more effective on apparent-size depth noise than smoothing it,
     * which just spreads the spike. 5 frames is ~2 frames of latency at 100+ fps. */
    std::deque<float> mcx, mcy, msp;
    auto median = [](std::deque<float> d) {
        std::sort(d.begin(), d.end());
        return d[d.size() / 2];
    };

    double t_pace = now_s();
    const double frame_budget = o.fps_cap > 0 ? 1.0 / (double)o.fps_cap : 0.0;
    while (!g_stop) {
        /* Pace the loop to fps_cap even if the camera ignores CAP_PROP_FPS: head
         * position barely changes, so running detection 100+ times/sec just burns
         * cores mirage needs. Sleep the remainder of this frame's budget. */
        if (frame_budget > 0.0) {
            double slack = frame_budget - (now_s() - t_pace);
            if (slack > 0.0) usleep((useconds_t)(slack * 1e6));
            t_pace = now_s();
        }
        if (!cap.read(frame) || frame.empty()) { usleep(5000); continue; }
        frames++;

        Head h{0,0,0,0,false};
        if (use_yunet) {
            cv::Mat faces;
            yunet->setInputSize(cv::Size(frame.cols, frame.rows));
            yunet->detect(frame, faces);
            /* rows: [x,y,w,h, reye(x,y), leye(x,y), nose, rmouth, lmouth, score];
             * pick the largest face by box area. */
            int best = -1; float best_area = 0;
            for (int i = 0; i < faces.rows; i++) {
                float w = faces.at<float>(i, 2), ht = faces.at<float>(i, 3);
                if (w * ht > best_area) { best_area = w * ht; best = i; }
            }
            if (best >= 0) {
                float rex = faces.at<float>(best, 4), rey = faces.at<float>(best, 5);
                float lex = faces.at<float>(best, 6), ley = faces.at<float>(best, 7);
                h.cx = 0.5f * (rex + lex);
                h.cy = 0.5f * (rey + ley);
                h.span_px = std::hypot(lex - rex, ley - rey);  /* interocular px */
                h.span_m  = 0.063f;                            /* avg IPD, metres */
                h.ok = h.span_px > 2.0f;
            }
        } else {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::equalizeHist(gray, gray);
            std::vector<cv::Rect> faces;
            haar.detectMultiScale(gray, faces, 1.2, 4, 0, cv::Size(80, 80));
            int best = -1; int best_area = 0;
            for (size_t i = 0; i < faces.size(); i++)
                if (faces[i].area() > best_area) { best_area = faces[i].area(); best = (int)i; }
            if (best >= 0) {
                h.cx = faces[best].x + faces[best].width  * 0.5f;
                h.cy = faces[best].y + faces[best].height * 0.5f;
                h.span_px = faces[best].width;   /* face width px */
                h.span_m  = 0.16f;               /* avg face width, metres */
                h.ok = h.span_px > 2.0f;
            }
        }

        if (!h.ok) continue;   /* no head this frame: mirage holds its last position */

        /* median-filter the raw measurement to outvote wrong-scale frames */
        mcx.push_back(h.cx); mcy.push_back(h.cy); msp.push_back(h.span_px);
        if (mcx.size() > 5) { mcx.pop_front(); mcy.pop_front(); msp.pop_front(); }
        float cx = median(mcx), cy = median(mcy), span = median(msp);

        /* metric recovery. distance from apparent size; lateral/vertical by similar
         * triangles. image +x is right, +y is DOWN -> negate both to get world
         * +right/+up. The horizontal sign also depends on whether the cam mirrors. */
        float Z = f_px * h.span_m / span;                 /* camera->head, metres */
        float xs = o.mirror ? +1.0f : -1.0f;
        float wx = xs * (cx - W * 0.5f) * Z / f_px;       /* world right (m) */
        float wy =     -(cy - H * 0.5f) * Z / f_px;       /* world up (m)    */

        /* backstop spike gate (the median already kills most): reject teleports that
         * exceed any plausible per-frame head move, snapping after a short streak so a
         * genuine re-lock recovers. Lateral in metres; depth as a ratio (apparent-size
         * noise is multiplicative). At 100+ fps a real move is tiny per frame, so the
         * gate can be tight without ever fighting an honest lean. */
        if (have_prev && reject_streak < 6) {
            float dxy = std::hypot(wx - pwx, wy - pwy);
            float zr  = (pZ > 1e-3f) ? Z / pZ : 1.0f;
            if (dxy > 0.06f || zr > 1.18f || zr < 0.847f) { reject_streak++; continue; }
        }
        reject_streak = 0;
        pwx = wx; pwy = wy; pZ = Z; have_prev = true;

        char msg[128];
        int n = std::snprintf(msg, sizeof msg, "{\"pos\":[%.4f,%.4f,%.4f]}", wx, wy, Z);
        if (sendto(fd, msg, n, 0, (struct sockaddr *)&dst, sizeof dst) < 0) {
            if (!warned_send) {  /* mirage may not have bound the socket yet */
                std::fprintf(stderr, "facecam: send failed (%s) - is mirage running? "
                             "retrying silently\n", std::strerror(errno));
                warned_send = true;
            }
        } else {
            warned_send = false;
        }

        if (o.verbose) {
            double t = now_s();
            if (t - t_log >= 1.0) {
                std::fprintf(stderr, "facecam: x=%+.3f y=%+.3f dist=%.3f m  %ld fps\n",
                             wx, wy, Z, frames);
                frames = 0; t_log = t;
            }
        }
    }

    close(fd);
    std::fprintf(stderr, "facecam: stopped\n");
    return 0;
}
