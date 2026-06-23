/*
 * viture_vio - drive VITURE's "Carina" VIO (OpenVINS) engine DIRECTLY, bypassing
 * libglasses' device-provider routing (which gates the Beast off the Carina path).
 *
 * Path-B harness.
 *   --init-only : just prove carina_vio_init() boots (config-schema work; no hardware).
 *   --run       : the real thing - feed live IMU (libglasses raw) + world-cam grayscale
 *                 frames into the engine and poll the 6DoF pose. Needs sudo (libusb +
 *                 cdc_acm unbind) and the Beast plugged in; stop the normal bridge first.
 *
 * KEY FACTS (hard-won, see memory viture-carina-anchor):
 *  - carina_vio_init's 1st string is the ENTIRE config as an inline yaml-cpp STRING
 *    (NOT a path); no `%YAML:1.0` header; must contain orb_database_path. 2nd string
 *    empty = pure VIO.
 *  - carina_vio_feed_imu(vector<float>{ax,ay,az, gx,gy,gz}, t_sec)  -- ACCEL FIRST,
 *    accel m/s^2, gyro rad/s, t in seconds (one monotonic clock shared with images).
 *  - carina_vio_feed_images2(gray0, gray1=NULL for mono, t_sec) -- GRAY8 w*h, size must
 *    equal config `resolution`.
 *  - carina_vio_get_gl_pose(float[32], t) -> Twb col-major[0..15], vel[16..18], angvel[19..21].
 *  - carina_vio_release() deadlocks on teardown -> we _exit().
 *
 * Build: make viture-vio
 * Run:   sudo -E LD_LIBRARY_PATH=viture-sdk VITURE_SDK=viture-sdk ./viture-vio --run -v
 */
#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <ctime>
#include <csignal>
#include <unistd.h>
#include <dlfcn.h>
#include <dirent.h>
#include <fcntl.h>

#include "camera.h"   /* cam_start / cam_acquire / cam_stop (V4L2 MJPEG -> RGB888) */

extern "C" {
    int  carina_vio_init(const std::string&, const std::string&);
    int  carina_vio_feed_imu(const std::vector<float>&, double);
    int  carina_vio_feed_images2(const char*, const char*, double);
    int  carina_vio_get_gl_pose(float*, double);
    int  carina_vio_get_imu_pose(float*, double);
    int  carina_vio_get_system_stage();
    int  carina_vio_get_slam_states_string(std::string&);
    int  carina_vio_reset_pose();
    int  carina_vio_release();
}

/* ---- libglasses ABI (raw IMU path; same as viture_bridge.c) ---- */
typedef void* XRH;
typedef void (*raw_cb)(float*, uint64_t, uint64_t);
#define VITURE_VID 0x35ca
#define BEAST_PID  0x1201
#define IMU_MODE_RAW 0
#define NATIVE_DOF_OFF 0
static XRH (*xr_create)(int);
static int (*xr_init)(XRH,const char*);
static int (*xr_start)(XRH);
static int (*xr_reg_raw)(XRH,raw_cb);
static int (*xr_open_imu)(XRH,uint8_t,uint8_t);
static int (*xr_set_dof)(XRH,int,int);
static int (*xr_get_dof)(XRH,int*,int*);
static int (*xr_stop)(XRH);
static int (*xr_shutdown)(XRH);
static void (*xr_destroy)(XRH);
static void (*xr_setlog)(int);

static volatile sig_atomic_t g_run = 1;
static void on_sigint(int){ g_run = 0; }
static double now_sec(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);
    return t.tv_sec + t.tv_nsec*1e-9; }

static int g_verbose = 0;
static long g_imu_n = 0, g_img_n = 0;
static double g_first_imu = 0;

/* IMU callback (libglasses thread): convert the Beast raw sample to Carina's expected
 * [ax,ay,az,gx,gy,gz] (accel m/s^2, gyro rad/s) and feed it. Beast raw layout: gyro xyz
 * (rad/s) [0..2], accel xyz (g) [3..5], mag [6..8], temp [9]. Timestamp = shared
 * monotonic clock (same now_sec() the camera uses) so IMU and images align. */
static void on_raw(float *d, uint64_t ts, uint64_t vsync){
    (void)ts; (void)vsync;
    std::vector<float> imu(6);
    imu[0] = d[3]*9.80665f; imu[1] = d[4]*9.80665f; imu[2] = d[5]*9.80665f;  /* accel m/s^2 */
    imu[3] = d[0];          imu[4] = d[1];          imu[5] = d[2];           /* gyro rad/s  */
    double t = now_sec();
    if (!g_first_imu) g_first_imu = t;
    carina_vio_feed_imu(imu, t);
    g_imu_n++;
}

/* Unbind cdc_acm from the Beast's CDC interfaces so libusb can claim them (root). */
static void detach_cdc_acm(void){
    DIR *d = opendir("/sys/bus/usb/devices"); if(!d) return;
    struct dirent *e; char p[512], buf[64];
    while((e=readdir(d))){
        if(strchr(e->d_name,':')) continue;
        snprintf(p,sizeof p,"/sys/bus/usb/devices/%s/idProduct",e->d_name);
        FILE*f=fopen(p,"r"); if(!f) continue; int pid=0; if(fscanf(f,"%x",&pid)!=1)pid=0; fclose(f);
        snprintf(p,sizeof p,"/sys/bus/usb/devices/%s/idVendor",e->d_name);
        f=fopen(p,"r"); int vid=0; if(f){ if(fscanf(f,"%x",&vid)!=1)vid=0; fclose(f);}
        if(vid!=VITURE_VID||pid!=BEAST_PID) continue;
        DIR*dd=opendir("/sys/bus/usb/devices"); struct dirent*ie;
        while(dd&&(ie=readdir(dd))){
            if(strncmp(ie->d_name,e->d_name,strlen(e->d_name))||!strchr(ie->d_name,':')) continue;
            snprintf(p,sizeof p,"/sys/bus/usb/devices/%s/driver",ie->d_name);
            ssize_t n=readlink(p,buf,sizeof buf-1); if(n<=0) continue; buf[n]=0;
            if(!strstr(buf,"cdc_acm")) continue;
            int u=open("/sys/bus/usb/drivers/cdc_acm/unbind",O_WRONLY);
            if(u>=0){ if(write(u,ie->d_name,strlen(ie->d_name))<0){} close(u);
                fprintf(stderr,"viture-vio: unbound cdc_acm from %s\n",ie->d_name); }
        }
        if(dd) closedir(dd);
    }
    closedir(d);
}

/* config = inline YAML (the whole thing) - see init notes above. Override w/ $VIO_CONFIG_FILE. */
static const char *CONFIG_YAML =
    "orb_database_path: \"\"\n"
    "verbosity: \"INFO\"\n"
    "use_fej: true\n"
    "integration: \"rk4\"\n"
    "use_stereo: false\n"
    "max_cameras: 1\n"
    "calib_cam_extrinsics: false\n"
    "calib_cam_intrinsics: false\n"
    "calib_cam_timeoffset: false\n"
    "max_clones: 11\n"
    "max_slam: 50\n"
    "max_slam_in_update: 25\n"
    "max_msckf_in_update: 40\n"
    "num_pts: 150\n"
    "fast_threshold: 20\n"
    "grid_x: 5\n"
    "grid_y: 5\n"
    "min_px_dist: 8\n"
    "feat_rep_msckf: \"GLOBAL_3D\"\n"
    "feat_rep_slam: \"ANCHORED_MSCKF_INVERSE_DEPTH\"\n"
    "dt_slam_delay: 1.0\n"
    "gravity_mag: 9.81\n"
    "init_window_time: 2.0\n"
    "init_imu_thresh: 1.0\n"
    "init_max_features: 50\n"
    "downsample_cameras: false\n"
    "calib_camimu_dt: 0.0\n"
    "gyroscope_noise_density: 0.0017\n"
    "gyroscope_random_walk: 0.00002\n"
    "accelerometer_noise_density: 0.02\n"
    "accelerometer_random_walk: 0.004\n"
    "imu0:\n"
    "  accelerometer_noise_density: 0.02\n"
    "  accelerometer_random_walk: 0.004\n"
    "  gyroscope_noise_density: 0.0017\n"
    "  gyroscope_random_walk: 0.00002\n"
    "  update_rate: 200.0\n"
    "cam0:\n"
    /* Cam<->IMU extrinsics. The KEY MUST BE `T_cam_imu` - NOT `T_imu_cam`. The binary
     * prints "parameter T_cam_imu not found, trying T_imu_cam instead" but that fallback
     * path leads to "Yaml cam0 T_cam_imu not ok" -> "Load Tbc fail!!" -> exit() (traced
     * to CameraUndistort.cc:393). Using `T_cam_imu` directly is accepted and the engine
     * reaches stage 1. FLAT 16-element sequence (getSequence<double> reads flat then
     * reshapes 4x4; nested rows make it try list->double -> throw). Identity placeholder -
     * replace with the real Beast cam->IMU mounting transform once calibrated. */
    "  T_cam_imu: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]\n"
    /* Beast uses a FISHEYE world camera - must use equidistant model, not pinhole!
     * Pinhole fell through CameraUndistort's model switch to its default branch, which
     * prints an error + calls exit() (traced in libcarina_vio: the accepted cases are
     * init_equidistant_with_fisheye_stereo_rectify and init_fov). Confirmed by the Mac
     * dump (mac-dump/notes.txt): the Beast world cam is Kannala-Brandt fisheye.
     * Intrinsics are ESTIMATED; calibrate with the checkerboard (viture-camcal) for real. */
    "  camera_model: equidistant\n"
    "  distortion_model: equidistant\n"
    "  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]\n"
    /* Fisheye intrinsics: lower focal length than pinhole due to wide FOV.
     * [fx, fy, cx, cy] - resolution MUST match the gray frames fed in --run (see --res). */
    "  intrinsics: [285.0, 285.0, 320.0, 240.0]\n"
    "  resolution: [640, 480]\n"
    "  cam_overlaps: []\n";

static std::string slurp(const char *path){
    FILE *f=fopen(path,"rb"); if(!f) return "";
    std::string s; char b[4096]; size_t n;
    while((n=fread(b,1,sizeof b,f))>0) s.append(b,n);
    fclose(f); return s;
}

/* dlopen libglasses + wire the raw-IMU entry points (mirrors viture_bridge.c). */
static void *load_libglasses(void){
    const char *env=getenv("VITURE_SDK");
    const char *dirs[]={ env, "viture-sdk", ".", "/usr/local/lib/viture" };
    char path[1024]; void*h=NULL;
    for(size_t i=0;i<sizeof dirs/sizeof*dirs;i++){
        if(!dirs[i]) continue;
        snprintf(path,sizeof path,"%s/libglasses.so",dirs[i]);
        h=dlopen(path,RTLD_NOW|RTLD_GLOBAL); if(h) break;
    }
    if(!h) h=dlopen("libglasses.so",RTLD_NOW|RTLD_GLOBAL);
    if(!h){ fprintf(stderr,"viture-vio: cannot load libglasses.so (%s)\n",dlerror()); return NULL; }
    xr_create=(XRH(*)(int))dlsym(h,"xr_device_provider_create");
    xr_init=(int(*)(XRH,const char*))dlsym(h,"xr_device_provider_initialize");
    xr_start=(int(*)(XRH))dlsym(h,"xr_device_provider_start");
    xr_reg_raw=(int(*)(XRH,raw_cb))dlsym(h,"register_raw_callback");
    xr_open_imu=(int(*)(XRH,uint8_t,uint8_t))dlsym(h,"open_imu");
    xr_set_dof=(int(*)(XRH,int,int))dlsym(h,"xr_device_provider_set_display_mode_and_native_dof");
    xr_get_dof=(int(*)(XRH,int*,int*))dlsym(h,"xr_device_provider_get_display_mode_and_native_dof");
    xr_stop=(int(*)(XRH))dlsym(h,"xr_device_provider_stop");
    xr_shutdown=(int(*)(XRH))dlsym(h,"xr_device_provider_shutdown");
    xr_destroy=(void(*)(XRH))dlsym(h,"xr_device_provider_destroy");
    xr_setlog=(void(*)(int))dlsym(h,"xr_device_provider_set_log_level");
    if(!xr_create||!xr_init||!xr_start||!xr_reg_raw||!xr_open_imu||!xr_set_dof){
        fprintf(stderr,"viture-vio: libglasses missing expected symbols\n"); return NULL; }
    return h;
}

int main(int argc, char **argv){
    bool init_only=false, run=false;
    const char *camdev="/dev/video1"; int camw=1280, camh=720;
    for(int i=1;i<argc;i++){
        if(!strcmp(argv[i],"--init-only")) init_only=true;
        else if(!strcmp(argv[i],"--run")) run=true;
        else if(!strcmp(argv[i],"--cam")&&i+1<argc) camdev=argv[++i];
        else if(!strcmp(argv[i],"--res")&&i+2<argc){ camw=atoi(argv[++i]); camh=atoi(argv[++i]); }
        else if(!strcmp(argv[i],"-v")||!strcmp(argv[i],"--verbose")) g_verbose=1;
        else { fprintf(stderr,"usage: %s [--init-only|--run] [--cam DEV] [--res W H] [-v]\n",argv[0]); return 2; }
    }
    if(!init_only && !run) init_only=true;

    std::string config = CONFIG_YAML;
    if(const char *cf=getenv("VIO_CONFIG_FILE")){ std::string s=slurp(cf);
        if(s.empty()){ fprintf(stderr,"viture-vio: VIO_CONFIG_FILE empty/missing\n"); return 1; }
        config=s; }
    std::string fusion=""; if(const char *fe=getenv("VIO_FUSION")) fusion=fe;
    { FILE*f=fopen("/tmp/viture-vio-config.yaml","wb"); if(f){ fwrite(config.data(),1,config.size(),f); fclose(f);} }

    fprintf(stderr,"viture-vio: carina_vio_init(<%zu-byte config>, <%zu-byte fusion>)...\n",
            config.size(),fusion.size());
    int r=-1;
    try { r=carina_vio_init(config,fusion); }
    catch(const std::exception&e){ fprintf(stderr,"viture-vio: init threw: %s\n",e.what()); return 1; }
    if(r!=0){ fprintf(stderr,"viture-vio: carina_vio_init -> %d (FAILED)\n",r); return 1; }
    fprintf(stderr,"viture-vio: *** ENGINE BOOTED *** (init -> 0)\n");

    if(init_only){
        /* The config is parsed ASYNC on a "System" thread (onStart); init returns 0 before
         * it runs. A bad key/type makes that thread throw -> terminate. So wait ~3s and
         * poll: if we survive, the config was ACCEPTED (no hardware needed to validate). */
        fprintf(stderr,"viture-vio: waiting for the async System thread to parse the config...\n");
        signal(SIGINT,on_sigint);
        for(int i=0;i<30 && g_run;i++){
            int st=carina_vio_get_system_stage();
            if(i%5==0) fprintf(stderr,"viture-vio:  t=%.1fs stage=%d\n", i*0.1, st);
            usleep(100000);
        }
        int st=carina_vio_get_system_stage(); float p[32]; int pr=carina_vio_get_gl_pose(p,0.0);
        fprintf(stderr,"viture-vio: *** CONFIG ACCEPTED *** survived async parse. stage=%d get_gl_pose->%d\n",st,pr);
        fprintf(stderr,"viture-vio: (stage/pose still -1 = waiting for IMU+camera data; that's --run)\n");
        _exit(0);
    }

    /* ---- --run: feed live IMU + camera ---- */
    signal(SIGINT,on_sigint); signal(SIGTERM,on_sigint);
    void *h=load_libglasses(); if(!h){ _exit(1); }
    if(xr_setlog) xr_setlog(1);
    detach_cdc_acm();
    XRH p=xr_create(BEAST_PID);
    if(!p){ fprintf(stderr,"viture-vio: provider create() failed (need sudo + cdc_acm unbound + glasses)\n"); _exit(1); }
    xr_reg_raw(p,on_raw);
    if(xr_init(p,NULL)!=0){ fprintf(stderr,"viture-vio: libglasses initialize failed\n"); _exit(1); }
    int dm=0x36,nd=0; if(xr_get_dof) xr_get_dof(p,&dm,&nd); if(dm<0) dm=0x36;
    xr_set_dof(p,dm,NATIVE_DOF_OFF);                 /* disable native 3DOF -> raw IMU streams */
    if(xr_open_imu(p,IMU_MODE_RAW,2)!=0) fprintf(stderr,"viture-vio: open_imu(RAW) failed\n");
    xr_start(p);
    fprintf(stderr,"viture-vio: IMU streaming. starting world cam %s @ %dx%d...\n",camdev,camw,camh);

    cam *c=cam_start(camdev,camw,camh);
    if(!c){ fprintf(stderr,"viture-vio: cam_start failed (try --cam /dev/videoN)\n"); _exit(1); }

    std::vector<uint8_t> gray((size_t)camw*camh);
    uint64_t seq=0; double last_log=0; int last_stage=-99;
    fprintf(stderr,"viture-vio: feeding engine. move the glasses (slowly pan/translate) to bootstrap VIO...\n");

    while(g_run){
        const uint8_t *rgb=nullptr; int w=0,h2=0;
        if(cam_acquire(c,&rgb,&w,&h2,&seq) && rgb){
            int n=w*h2;
            if((int)gray.size()<n) gray.resize(n);
            /* RGB888 -> GRAY8 (luma); 77+150+29=256 */
            for(int i=0;i<n;i++){ const uint8_t*px=rgb+3*i;
                gray[i]=(uint8_t)((px[0]*77+px[1]*150+px[2]*29)>>8); }
            carina_vio_feed_images2((const char*)gray.data(), nullptr, now_sec());
            g_img_n++;
        }
        int stage=carina_vio_get_system_stage();
        float pose[32]; memset(pose,0,sizeof pose);
        int pr=carina_vio_get_gl_pose(pose,0.0);

        double t=now_sec();
        if(stage!=last_stage){
            std::string ss; carina_vio_get_slam_states_string(ss);
            fprintf(stderr,"viture-vio: *** stage %d -> %d *** %s\n",last_stage,stage,ss.c_str());
            last_stage=stage;
        }
        if(t-last_log>=0.5){
            fprintf(stderr,"viture-vio: imu=%ld img=%ld stage=%d pose->%d | t=[%.3f %.3f %.3f]\n",
                    g_imu_n,g_img_n,stage,pr, pose[12],pose[13],pose[14]);  /* col-major Twb translation = elems 12,13,14 */
            last_log=t;
        }
        usleep(3000);
    }

    fprintf(stderr,"\nviture-vio: stopping (imu=%ld img=%ld).\n",g_imu_n,g_img_n);
    cam_stop(c);
    if(xr_stop) xr_stop(p);
    if(xr_shutdown) xr_shutdown(p);
    if(xr_destroy) xr_destroy(p);
    /* skip carina_vio_release() - it deadlocks; _exit past it */
    _exit(0);
}
