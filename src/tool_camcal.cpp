/* tool_camcal.cpp - headless intrinsic calibration of the VITURE Beast world camera.
 *
 * The Beast won't hand out its factory camera calibration over USB (get_calibration_config
 * returns -3 for every blob), and self-VIO needs real lens intrinsics - guessing the FOV
 * makes the visual rotation prediction garbage. So we calibrate the camera ourselves the
 * standard way: show it a printed checkerboard from many angles, run cv::calibrateCamera.
 *
 * Reuses camera.cpp (the same MJPEG capture mirage uses). Fully headless: it auto-captures
 * a view whenever it sees the full board AND the board has moved/zoomed enough from every
 * previously kept view (so the set spans the frame + a range of distances/tilts - what a
 * good calibration needs). Prints progress; when it has enough, it calibrates and writes
 *   viture-cam.calib   (fx fy cx cy + distortion + image size + derived HFOV)
 * which mirage/worldvio load to fix the focal length.
 *
 *   make camcal
 *   ./viture-camcal [cols rows]     # cols rows = INNER corners (default 9 6)
 *
 * Print any checkerboard (e.g. the OpenCV 9x6 sample); square size is irrelevant to the
 * intrinsics we need, so it isn't asked for. Hold it ~0.3-1 m away, fill the frame, and
 * slowly sweep it across the view and tilt it until it says "done".
 */
#include "camera.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <ctime>
#include <vector>
#include <cmath>

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int){ g_stop = 1; }

static double mono(){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec + t.tv_nsec*1e-9; }

/* How far (fraction of frame width) a board's centre or scale must differ from EVERY kept
 * view to count as a new pose - keeps the set diverse instead of 20 near-identical frames. */
static const float DIVERSITY = 0.08f;
static const int   TARGET_VIEWS = 18;

int main(int argc, char **argv){
    int cols = 9, rows = 6;                 /* inner corners */
    if (argc >= 3){ cols = atoi(argv[1]); rows = atoi(argv[2]); }
    if (cols < 3 || rows < 3){ fprintf(stderr, "bad board %dx%d\n", cols, rows); return 2; }
    signal(SIGINT, on_sigint);
    cv::Size board(cols, rows);
    printf("camcal: checkerboard %dx%d inner corners; target %d diverse views.\n", cols, rows, TARGET_VIEWS);

    char dev[64];
    if (!cam_find(dev, sizeof dev)){ fprintf(stderr, "camcal: no world camera found\n"); return 1; }
    cam *c = cam_start(dev, 1280, 720);
    if (!c){ fprintf(stderr, "camcal: cam_start failed\n"); return 1; }

    /* board object points (Z=0 plane, unit squares - scale is irrelevant for intrinsics) */
    std::vector<cv::Point3f> objp;
    for (int r = 0; r < rows; r++) for (int cc = 0; cc < cols; cc++) objp.emplace_back((float)cc, (float)r, 0.0f);

    std::vector<std::vector<cv::Point3f>> obj_views;
    std::vector<std::vector<cv::Point2f>> img_views;
    std::vector<cv::Point2f> kept_centres;          /* normalised (by width) centre+scale of kept views */
    std::vector<float>       kept_scales;

    const uint8_t *rgb = nullptr; int w = 0, h = 0; uint64_t seq = 0;
    int imgw = 0, imgh = 0;
    double t_last_msg = 0; long frames = 0; bool last_found = false;

    printf("camcal: hold the board in view and sweep it around slowly...\n");
    while (!g_stop && (int)img_views.size() < TARGET_VIEWS){
        if (cam_failed(c)){ fprintf(stderr, "camcal: camera died (Beast renumbered?)\n"); break; }
        if (!cam_acquire(c, &rgb, &w, &h, &seq)){ struct timespec ts{0, 5'000'000}; nanosleep(&ts, nullptr); continue; }
        imgw = w; imgh = h; frames++;

        cv::Mat rgbm(h, w, CV_8UC3, (void*)rgb);
        cv::Mat gray; cv::cvtColor(rgbm, gray, cv::COLOR_RGB2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, board, corners,
                        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
        last_found = found;
        if (found){
            cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.01));
            /* centre + scale (bbox diagonal) of this detection, normalised by frame width */
            cv::Point2f mn(1e9f,1e9f), mx(-1e9f,-1e9f), ctr(0,0);
            for (auto &p : corners){ ctr += p; mn.x=std::min(mn.x,p.x); mn.y=std::min(mn.y,p.y);
                                     mx.x=std::max(mx.x,p.x); mx.y=std::max(mx.y,p.y); }
            ctr *= 1.0f/corners.size();
            cv::Point2f cn(ctr.x/w, ctr.y/w);
            float scale = (float)cv::norm(mx-mn)/w;
            bool diverse = true;
            for (size_t i=0;i<kept_centres.size();i++){
                if (cv::norm(cn-kept_centres[i]) < DIVERSITY && fabsf(scale-kept_scales[i]) < DIVERSITY){ diverse=false; break; }
            }
            if (diverse){
                obj_views.push_back(objp); img_views.push_back(corners);
                kept_centres.push_back(cn); kept_scales.push_back(scale);
                printf("camcal: captured view %d/%d  (centre %.2f,%.2f scale %.2f)\n",
                       (int)img_views.size(), TARGET_VIEWS, cn.x, cn.y, scale);
                fflush(stdout);
            }
        }
        if (mono() - t_last_msg > 1.5){
            t_last_msg = mono();
            printf("camcal: %s ... %d/%d kept (%.0f fps)\r", last_found?"board VISIBLE - move it a bit":"looking for board",
                   (int)img_views.size(), TARGET_VIEWS, frames/1.5); fflush(stdout);
            frames = 0;
        }
    }
    printf("\n");
    cam_stop(c);

    if ((int)img_views.size() < 6){
        fprintf(stderr, "camcal: only %zu views - need >=6. Aborting (no file written).\n", img_views.size());
        return 1;
    }

    printf("camcal: calibrating on %zu views (%dx%d)...\n", img_views.size(), imgw, imgh);
    cv::Mat K = cv::Mat::eye(3,3,CV_64F), dist;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(obj_views, img_views, cv::Size(imgw,imgh), K, dist, rvecs, tvecs,
                    cv::CALIB_RATIONAL_MODEL);   /* fisheye-ish wide UVC lens -> rational distortion */

    double fx = K.at<double>(0,0), fy = K.at<double>(1,1);
    double cx = K.at<double>(0,2), cy = K.at<double>(1,2);
    double hfov = 2.0 * atan2((double)imgw * 0.5, fx) * 180.0 / M_PI;
    double vfov = 2.0 * atan2((double)imgh * 0.5, fy) * 180.0 / M_PI;

    printf("\n=== calibration result (RMS reproj error %.3f px) ===\n", rms);
    printf("  fx=%.2f fy=%.2f  cx=%.2f cy=%.2f   (%dx%d)\n", fx, fy, cx, cy, imgw, imgh);
    printf("  HFOV=%.1f deg  VFOV=%.1f deg   <-- this replaces worldvio's 70deg guess\n", hfov, vfov);
    printf("  dist[");
    for (int i=0;i<dist.cols*dist.rows;i++) printf("%s%.4f", i?",":"", dist.at<double>(i));
    printf("]\n");
    if (rms > 1.5) printf("  WARNING: RMS %.2f is high - recapture with the board sharper/more spread.\n", rms);

    FILE *f = fopen("viture-cam.calib", "w");
    if (!f){ perror("camcal: open viture-cam.calib"); return 1; }
    fprintf(f, "# VITURE Beast world-cam intrinsics (self-calibrated, cv::calibrateCamera)\n");
    fprintf(f, "width %d\nheight %d\n", imgw, imgh);
    fprintf(f, "fx %.6f\nfy %.6f\ncx %.6f\ncy %.6f\n", fx, fy, cx, cy);
    fprintf(f, "hfov_deg %.6f\nvfov_deg %.6f\n", hfov, vfov);
    fprintf(f, "rms %.6f\nviews %zu\n", rms, img_views.size());
    fprintf(f, "dist");
    for (int i=0;i<dist.cols*dist.rows;i++) fprintf(f, " %.8f", dist.at<double>(i));
    fprintf(f, "\n");
    fclose(f);
    printf("camcal: wrote viture-cam.calib\n");
    return 0;
}
