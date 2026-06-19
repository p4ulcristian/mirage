/*
 * viture-bridge - feed VITURE (Beast) head tracking into mirage.
 *
 * The Beast (35ca:1201) does NOT speak the old One/Pro HID protocol. It's a
 * "Gen2" device whose control + IMU ride a CDC-ACM channel, driven by VITURE's
 * v2.0.0 SDK (libglasses.so). That SDK only ships as a VITURE-distributed binary
 * (no public download) but DOES have a native aarch64 build - bundled in
 * wheaney/XRLinuxDriver under lib/aarch64/viture/ - so it runs on the M2/Asahi
 * box mirage targets. We dlopen() it, pull RAW gyro/accel/mag, fuse with the
 * same Madgwick AHRS as the RayNeo path (rayneo.c), and emit the orientation as
 * the quaternion UDP packet pose.cpp already reads. mirage is untouched.
 *
 * THREE things the SDK needs that aren't obvious (all handled here):
 *   1. cdc_acm owns the Beast's control interfaces - we unbind it first (sysfs),
 *      so libusb inside the SDK can claim them. Needs root (so does libusb's
 *      /dev/bus/usb access) - run via scripts/viture-bridge.sh (sudo).
 *   2. The Beast's NATIVE on-glasses 3DOF must be disabled
 *      (set_display_mode_and_native_dof(mode, DOF_0)) or it won't stream raw IMU.
 *   3. open_imu(MODE_POSE) is rejected by the Beast; MODE_RAW works - so we fuse
 *      ourselves.
 *
 * Run:  VITURE_SDK=/path/to/lib/aarch64/viture sudo -E ./viture-bridge -v
 *
 * Axis tuning: the sensor frame differs from RayNeo's, so --qmap permutes/negates
 * the output quaternion at runtime (e.g. --qmap w,-y,-z,-x) until head motion
 * maps 1:1 in mirage; bake the winner into the default below.
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <fcntl.h>
#include <dirent.h>
#include <unistd.h>
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "rayneo.h"   /* reuse rayneo_imu + the legacy Madgwick AHRS (A/B fallback) */
#include "vqf_shim.h" /* VQF: modern AHRS w/ runtime gyro-bias est + accel/mag rejection */

#define VITURE_VID  0x35ca
#define BEAST_PID   0x1201

/* ---- VITURE v2.0.0 SDK ABI (from XRLinuxDriver's viture headers) ---- */
typedef void* XRH;
typedef void (*raw_cb)(float *data, uint64_t ts, uint64_t vsync);
/* onboard fused-pose callback: SetImuPoseCallback(void(*)(float*, unsigned long)) */
typedef void (*pose_cb)(float *pose, unsigned long ts);
/* IMU raw layout (Beast/Luma): gyro xyz, accel xyz, mag xyz, temp (10 floats) */
#define IMU_MODE_RAW   0
#define IMU_MODE_POSE  1
#define NATIVE_DOF_OFF 0

static XRH (*xr_create)(int);
static int (*xr_init)(XRH, const char*);
static int (*xr_start)(XRH);
static int (*xr_reg_raw)(XRH, raw_cb);
static int (*xr_open_imu)(XRH, uint8_t, uint8_t);
static int (*xr_set_dof)(XRH, int, int);
static int (*xr_get_dof)(XRH, int*, int*);
static int (*xr_valid)(int);
static void (*xr_setlog)(int);
static int (*xr_stop)(XRH);
static int (*xr_shutdown)(XRH);
static void (*xr_destroy)(XRH);
static int (*xr_set_display_mode)(XRH, int);
static int (*xr_get_display_mode)(XRH);
/* pose / VIO probe (does the Beast expose a fused or 6DoF pose?) */
static void (*xr_reg_pose)(pose_cb);            /* register_pose_callback */
static int  (*xr_get_gl_pose)(XRH, float*, double);   /* get_gl_pose_carina(handle,out,ts) */
static int  g_probe_gl = 0;
static long g_pose_n = 0;
static double g_pose_log = 0;
/* Direct Carina-engine probes. libcarina_vio.so is a NEEDED dep of libglasses.so,
 * which we dlopen with RTLD_GLOBAL, so these C-linkage exports resolve through the
 * libglasses handle. They are VITURE's *own* fusion outputs - the thing that makes
 * their anchor rock-solid - which we currently throw away in favour of our Madgwick:
 *   get_imu_pose : factory-calibrated IMU filter (Tier-1 anchor, no camera needed)
 *   get_gl_pose  : 6DOF GL transform (Tier-2 VIO, if the a1088/VIO path is live)
 * Signatures from objdump+c++filt: carina_api::get_imu_pose(float*,double),
 * get_gl_pose(float*,double). We pass a 16-float buffer (room for a 4x4) to be safe. */
static int (*ca_imu_pose)(float*, double);
static int (*ca_gl_pose)(float*, double);
static double g_capose_log = 0;

/* High-level libglasses Carina path (what XRLinuxDriver uses for 6DoF). The SDK's
 * CarinaDeviceProvider reads the device's factory calibration + drives the cameras +
 * runs the VIO internally - the app passes NO config. We just create->initialize->
 * register_callbacks_carina->start, then poll get_gl_pose_carina. The whole question
 * is whether the Beast reports device_type == CARINA so this path is available. */
static double now_sec(void);   /* fwd decl (defined below; used by on_carina_pose) */
typedef void (*carina_pose_cb)(float*, double);
typedef void (*carina_vsync_cb)(double);
typedef void (*carina_imu_cb)(float*, double);
typedef void (*carina_cam_cb)(char*,char*,char*,char*,double,int,int);
static int (*xr_reg_carina)(XRH, carina_pose_cb, carina_vsync_cb, carina_imu_cb, carina_cam_cb);
static int (*xr_get_devtype)(XRH);
static int (*xr_reset_carina)(XRH);
static long g_cpose_n = 0; static double g_cpose_log = 0;
static void on_carina_pose(float *pose, double ts){
    (void)ts; g_cpose_n++;
    double t = now_sec();
    if (t - g_cpose_log >= 0.3){ g_cpose_log = t;
        fprintf(stderr,"CARINA-POSE n=%ld | [0..8] % .4f % .4f % .4f | % .4f % .4f % .4f % .4f | % .4f % .4f\n",
                g_cpose_n, pose[0],pose[1],pose[2], pose[3],pose[4],pose[5],pose[6], pose[7],pose[8]);
    }
}

static volatile sig_atomic_t g_run = 1;
static int g_stall = 0;           /* set when the watchdog trips -> re-exec to recover */
static int g_cold  = 0;           /* set when we NEVER streamed -> USB-reset before re-exec */
static char **g_argv = NULL;      /* saved for self-re-exec on stall */
static void on_sigint(int s){ (void)s; g_run = 0; }
static volatile sig_atomic_t g_reload = 0;
static void on_hup(int s){ (void)s; g_reload = 1; }  /* re-read /tmp/viture-qmap live */
static double now_sec(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);
    return t.tv_sec + t.tv_nsec*1e-9; }

/* ---- shared with the SDK's IMU callback thread ---- */
static rayneo_ahrs g_ahrs;
static vqf_handle *g_vqf = NULL;     /* VQF filter (default); NULL falls back to Madgwick */
static int g_use_vqf = 1;            /* --madgwick sets 0 for A/B against the old filter */
static int g_sock; static struct sockaddr_in g_dst;
static int g_qi[4]; static double g_qs[4];      /* output quaternion remap */
/* input axis remap, Beast sensor->AHRS frame. Default = "-z,x,-y" (forward<-(-z),
 * left-right<-(+x), up<-(-y)): verified on the Beast so head motion tracks 1:1 with
 * mirage's RayNeo-tuned device->world remap. Derive afresh with viture-calibrate.py. */
static int g_mi[3]={2,0,1}; static float g_ms[3]={-1,1,-1};
static double g_gyro_scale = 1.0;                /* Beast gyro is already rad/s (peak ~7 = ~420 deg/s) */
static float g_dt = 1.0f/120.0f;                 /* nominal sample dt from the SDK rate; superseded at runtime by g_dt_meas */
/* Measured-rate dt: the nominal g_dt is usually WRONG (Beast streams ~124Hz, not 120) and a
 * few-% dt scale error drifts the integrator AND skews VQF's rest/bias time-windows - one bad
 * constant degrades drift, tracking lag and jitter at once. We measure the TRUE mean rate from
 * the monotonic clock over a 1s window and feed THAT (still effectively constant, so no
 * per-sample batching tremor, but the correct constant). g_prev_ts tracks the device timestamp
 * delta for diagnostics - if that clock proves clean, a future per-sample dt is even better. */
static double g_rate_t0 = 0; static long g_rate_n0 = 0; static float g_dt_meas = 0;
static uint64_t g_prev_ts = 0; static double g_ts_d_ema = 0;
static int g_verbose, g_use_mag;
static int g_seeded = 0;
static float g_gyro_bias[3] = {0,0,0};
/* Live magnetometer span tracker (raw sensor frame d[6..8]). If the mag is real,
 * each axis min/max spreads as you rotate the glasses (do a slow figure-8); the
 * centre (min+max)/2 is the hard-iron bias, the half-span the per-axis scale.
 * If the spans stay ~0, the Beast isn't streaming mag and Path A is off. */
static float g_mag_min[3]={1e9f,1e9f,1e9f}, g_mag_max[3]={-1e9f,-1e9f,-1e9f};
static long g_n = 0;
static double g_last_log = 0;
static double g_last_raw = 0;     /* monotonic time of the last raw IMU callback (stall watchdog) */

/* rotate vector v by unit quaternion q (w,x,y,z): out = q * v * conj(q) */
static void qrot(const double q[4], const double v[3], double out[3]){
    double w=q[0],x=q[1],y=q[2],z=q[3];
    double tx=2.0*(y*v[2]-z*v[1]), ty=2.0*(z*v[0]-x*v[2]), tz=2.0*(x*v[1]-y*v[0]);
    out[0]=v[0]+w*tx+(y*tz-z*ty);
    out[1]=v[1]+w*ty+(z*tx-x*tz);
    out[2]=v[2]+w*tz+(x*ty-y*tx);
}

static void on_raw(float *d, uint64_t ts, uint64_t vsync){
    (void)vsync;
    double tw = now_sec();

    /* Measure the true mean sample rate over a 1s window -> dt. Constant within the window
     * (no per-sample wall-clock jitter, which IS the tremor under batched delivery), but the
     * constant is now CORRECT instead of a hard-coded guess. EMA across windows so it settles
     * smoothly and tracks any slow rate change. */
    if (g_rate_t0 == 0){ g_rate_t0 = tw; g_rate_n0 = g_n; }
    else if (tw - g_rate_t0 >= 1.0){
        double r = (double)(g_n - g_rate_n0) / (tw - g_rate_t0);
        if (r > 30.0 && r < 2000.0){               /* sane IMU-rate band */
            float dtm = (float)(1.0/r);
            g_dt_meas = (g_dt_meas > 0) ? 0.5f*g_dt_meas + 0.5f*dtm : dtm;
        }
        g_rate_t0 = tw; g_rate_n0 = g_n;
    }
    float dt = (g_dt_meas > 0) ? g_dt_meas : g_dt;

    /* Device-clock delta (diagnostic only - unit unknown until we read the log). If this is a
     * clean monotonic MCU timestamp it's the ideal per-sample dt source for a later refinement. */
    uint64_t ts_d = (g_prev_ts && ts > g_prev_ts) ? ts - g_prev_ts : 0;
    g_prev_ts = ts;
    if (ts_d) g_ts_d_ema = g_ts_d_ema>0 ? 0.99*g_ts_d_ema + 0.01*(double)ts_d : (double)ts_d;

    /* input axis remap (Beast sensor frame -> AHRS frame): apply the same
     * permutation/signs to gyro, accel and mag so the fused frame is consistent. */
    rayneo_imu s = {0};
    for (int i=0;i<3;i++){
        int j=g_mi[i]; float sg=g_ms[i];
        s.gyro_rad[i] = sg * d[j]    * (float)g_gyro_scale;
        s.accel[i]    = sg * d[3+j];
        s.mag[i]      = sg * d[6+j];
    }

    double src[4];
    if (g_use_vqf){
        /* VQF does its own gravity init, continuous gyro-bias estimation, filtered tilt
         * correction, and (9-axis) magnetic-disturbance rejection - so none of the manual
         * seed/bias hacks the Madgwick path needs apply. Feed gyro rad/s + accel m/s^2
         * (Beast accel is in g) [+ mag in the same remapped frame]. */
        double gyr[3] = { s.gyro_rad[0], s.gyro_rad[1], s.gyro_rad[2] };
        double acc[3] = { s.accel[0]*9.80665, s.accel[1]*9.80665, s.accel[2]*9.80665 };
        if (g_use_mag){ double mag[3]={s.mag[0],s.mag[1],s.mag[2]};
            vqf_update9(g_vqf,gyr,acc,mag); vqf_quat9(g_vqf,src); }
        else { vqf_update6(g_vqf,gyr,acc); vqf_quat6(g_vqf,src); }
    } else {
        /* legacy Madgwick A/B path (--madgwick): seed to gravity, auto-zero gyro bias, fuse. */
        if (!g_seeded){
            rayneo_imu g = s; g.gyro_rad[0]=g.gyro_rad[1]=g.gyro_rad[2]=0;
            float sb=g_ahrs.beta; g_ahrs.beta=2.0f;
            for (int it=0; it<400; it++) rayneo_ahrs_update(&g_ahrs,&g,0.01f);
            g_ahrs.beta=sb; g_seeded=1;
        }
        float gm = sqrtf(s.gyro_rad[0]*s.gyro_rad[0]+s.gyro_rad[1]*s.gyro_rad[1]+s.gyro_rad[2]*s.gyro_rad[2]);
        if (gm < 2.5f*(float)(M_PI/180.0)){
            float lr = dt/1.5f; if (lr>0.05f) lr=0.05f;
            for (int k=0;k<3;k++) g_gyro_bias[k]+=lr*(s.gyro_rad[k]-g_gyro_bias[k]);
        }
        for (int k=0;k<3;k++) s.gyro_rad[k]-=g_gyro_bias[k];
        if (g_use_mag) rayneo_ahrs_update9(&g_ahrs,&s,s.mag,dt);
        else           rayneo_ahrs_update(&g_ahrs,&s,dt);
        src[0]=g_ahrs.q[0]; src[1]=g_ahrs.q[1]; src[2]=g_ahrs.q[2]; src[3]=g_ahrs.q[3];
    }

    double out[4]; for (int k=0;k<4;k++) out[k]=g_qs[k]*src[g_qi[k]];
    double nm = sqrt(out[0]*out[0]+out[1]*out[1]+out[2]*out[2]+out[3]*out[3]);
    if (nm>1e-9) for(int k=0;k<4;k++) out[k]/=nm;

    /* World-frame LINEAR acceleration (gravity removed) for mirage's VIO position fusion.
     * Rotate the sensor accel (g -> m/s^2) by the OUTPUT quaternion (sensor -> mirage
     * world, so it matches `head`), then high-pass out gravity: a slow LP tracks the
     * constant world gravity vector, the residual is linear accel - frame-agnostic, no
     * fixed up-axis needed. Packet grows to 7 doubles: quat[0..3] + lin-accel[4..6];
     * mirage reads whichever size arrives (back-compatible). (Correct for an identity
     * qmap, which is the Beast default; a non-identity qmap would need the matching
     * vector relabel - noted, not needed here.) */
    double pkt[7] = { out[0], out[1], out[2], out[3], 0, 0, 0 };
    {
        double a_s[3] = { s.accel[0]*9.80665, s.accel[1]*9.80665, s.accel[2]*9.80665 };
        double a_w[3]; qrot(out, a_s, a_w);
        static double grav[3] = {0,0,0}; static int gi = 0;
        if (!gi){ grav[0]=a_w[0]; grav[1]=a_w[1]; grav[2]=a_w[2]; gi=1; }
        double ga = (double)dt/0.5; if (ga>0.08) ga=0.08;   /* ~0.5s gravity tracker */
        for (int k=0;k<3;k++) grav[k] += ga*(a_w[k]-grav[k]);
        pkt[4]=a_w[0]-grav[0]; pkt[5]=a_w[1]-grav[1]; pkt[6]=a_w[2]-grav[2];
    }
    sendto(g_sock,pkt,sizeof pkt,0,(struct sockaddr*)&g_dst,sizeof g_dst);
    g_n++;

    /* track raw mag span (sensor frame) so we can see if the mag is real + get its range */
    for (int k=0;k<3;k++){ float m=d[6+k];
        if (m<g_mag_min[k]) g_mag_min[k]=m;
        if (m>g_mag_max[k]) g_mag_max[k]=m; }

    double t = tw;
    g_last_raw = t;               /* feed the stall watchdog */
    if (g_verbose && t-g_last_log>=0.25){
        /* euler from the fused (pre-qmap) quaternion - same for either filter */
        rayneo_ahrs tmp; tmp.q[0]=src[0]; tmp.q[1]=src[1]; tmp.q[2]=src[2]; tmp.q[3]=src[3];
        float yaw,pitch,roll; rayneo_ahrs_euler(&tmp,&yaw,&pitch,&roll);
        if (g_use_vqf){
            double bias[3]={0,0,0}; vqf_get_bias(g_vqf,bias);
            fprintf(stderr,"[VQF%s] ypr % 7.1f % 7.1f % 7.1f | bias(deg/s) % 5.2f % 5.2f % 5.2f | rest %d magrej %d | rate %.1fHz tsd %.0f | n %ld\n",
                    g_use_mag?"+mag":"", yaw,pitch,roll,
                    bias[0]*57.2958,bias[1]*57.2958,bias[2]*57.2958,
                    vqf_rest_detected(g_vqf), vqf_mag_dist_detected(g_vqf),
                    g_dt_meas>0?1.0/g_dt_meas:0.0, g_ts_d_ema, g_n);
        } else {
            float mm = sqrtf(d[6]*d[6]+d[7]*d[7]+d[8]*d[8]);
            fprintf(stderr,"[MADG] ypr % 7.1f % 7.1f % 7.1f | acc % 6.2f % 6.2f % 6.2f | gyr % 6.2f % 6.2f % 6.2f "
                    "| MAG % 7.1f % 7.1f % 7.1f |m|% 6.1f span[% 6.1f % 6.1f % 6.1f] n %ld\n",
                    yaw,pitch,roll, d[3],d[4],d[5], d[0],d[1],d[2], d[6],d[7],d[8], mm,
                    g_mag_max[0]-g_mag_min[0], g_mag_max[1]-g_mag_min[1], g_mag_max[2]-g_mag_min[2], g_n);
        }
        g_last_log=t;
    }
}

/* PROBE: the SDK's onboard fused-pose callback. We don't know the field count, so we
 * log the first 7 floats - if it's a 3DoF quaternion only, [0..3] move and [4..6] are
 * static/garbage; if it's 6DoF, [4..6] track lateral/forward translation as you lean.
 * Harmless if the Beast never emits pose (this just never fires). */
static void on_pose(float *pse, unsigned long ts){
    (void)ts; g_pose_n++;
    double t = now_sec();
    if (t - g_pose_log >= 0.5){
        fprintf(stderr,"POSE n %ld | [0..6] % .4f % .4f % .4f % .4f | % .4f % .4f % .4f\n",
                g_pose_n, pse[0],pse[1],pse[2],pse[3], pse[4],pse[5],pse[6]);
        g_pose_log = t;
    }
}

/* Unbind cdc_acm from the Beast's CDC interfaces so the SDK's libusb can claim
 * them. Scans sysfs for the 35ca:1201 device and unbinds every interface bound
 * to cdc_acm. Needs root; harmless (and a no-op) if already unbound. */
static void detach_cdc_acm(void){
    DIR *d = opendir("/sys/bus/usb/devices"); if (!d) return;
    struct dirent *e; char p[512], buf[64];
    while ((e=readdir(d))){
        if (strchr(e->d_name,':')) continue;               /* want devices, not interfaces */
        snprintf(p,sizeof p,"/sys/bus/usb/devices/%s/idProduct",e->d_name);
        FILE *f=fopen(p,"r"); if(!f) continue;
        int pid=0; if(fscanf(f,"%x",&pid)!=1) pid=0; fclose(f);
        snprintf(p,sizeof p,"/sys/bus/usb/devices/%s/idVendor",e->d_name);
        f=fopen(p,"r"); int vid=0; if(f){ if(fscanf(f,"%x",&vid)!=1) vid=0; fclose(f);}
        if (vid!=VITURE_VID || pid!=BEAST_PID) continue;
        /* iterate this device's interfaces (name:cfg.intf) */
        DIR *dd=opendir("/sys/bus/usb/devices"); struct dirent *ie;
        while (dd && (ie=readdir(dd))){
            if (strncmp(ie->d_name,e->d_name,strlen(e->d_name)) || !strchr(ie->d_name,':')) continue;
            snprintf(p,sizeof p,"/sys/bus/usb/devices/%s/driver",ie->d_name);
            ssize_t n=readlink(p,buf,sizeof buf-1); if(n<=0) continue; buf[n]=0;
            if (!strstr(buf,"cdc_acm")) continue;
            int u=open("/sys/bus/usb/drivers/cdc_acm/unbind",O_WRONLY);
            if (u>=0){ if(write(u,ie->d_name,strlen(ie->d_name))<0){} close(u);
                fprintf(stderr,"viture-bridge: unbound cdc_acm from %s\n",ie->d_name); }
        }
        if (dd) closedir(dd);
    }
    closedir(d);
}

/* USB-level reset of the Beast (USBDEVFS_RESET) - the software equivalent of a
 * physical replug. Clears the cold "Failed to send USB command -1" wedge where
 * cdc_acm/libusb left the control pipe stuck and open_imu() can't get through.
 * Finds the 35ca:1201 bus/dev via sysfs and ioctls its /dev/bus/usb node. DP
 * video rides a separate USB-C alt-mode lane (not this USB data device), so the
 * display stays up across the reset. Needs root; harmless no-op if not found. */
static void usb_reset_beast(void){
    DIR *d = opendir("/sys/bus/usb/devices"); if(!d) return;
    struct dirent *e; char p[512]; int done=0;
    while(!done && (e=readdir(d))){
        if(strchr(e->d_name,':')) continue;               /* devices, not interfaces */
        snprintf(p,sizeof p,"/sys/bus/usb/devices/%s/idProduct",e->d_name);
        FILE *f=fopen(p,"r"); if(!f) continue;
        int pid=0; if(fscanf(f,"%x",&pid)!=1) pid=0; fclose(f);
        snprintf(p,sizeof p,"/sys/bus/usb/devices/%s/idVendor",e->d_name);
        f=fopen(p,"r"); int vid=0; if(f){ if(fscanf(f,"%x",&vid)!=1) vid=0; fclose(f);}
        if(vid!=VITURE_VID || pid!=BEAST_PID) continue;
        int bus=0,dev=0;
        snprintf(p,sizeof p,"/sys/bus/usb/devices/%s/busnum",e->d_name);
        f=fopen(p,"r"); if(f){ if(fscanf(f,"%d",&bus)!=1) bus=0; fclose(f);}
        snprintf(p,sizeof p,"/sys/bus/usb/devices/%s/devnum",e->d_name);
        f=fopen(p,"r"); if(f){ if(fscanf(f,"%d",&dev)!=1) dev=0; fclose(f);}
        if(bus>0 && dev>0){
            char node[64]; snprintf(node,sizeof node,"/dev/bus/usb/%03d/%03d",bus,dev);
            int fd=open(node,O_WRONLY);
            if(fd>=0){ int r=ioctl(fd,USBDEVFS_RESET,0);
                fprintf(stderr,"viture-bridge: USBDEVFS_RESET %s -> %d\n",node,r);
                close(fd); done=1; }
            else fprintf(stderr,"viture-bridge: open %s failed (need root for USB reset)\n",node);
        }
    }
    closedir(d);
}

/* Bounded self-recovery: optionally USB-reset the Beast, back off, then re-exec
 * ourselves to retry the whole bring-up from scratch (same PID + name, so the
 * pgrep/pkill -x the launcher relies on still work). Used for BOTH the cold
 * open_imu wedge and a failed create() (USB not enumerated yet right after boot).
 * The attempt count rides across re-execs in $VB_TRY and is cleared once samples
 * flow; capped so a genuinely unplugged Beast eventually gives up instead of
 * spinning forever. Only returns if execv fails or the cap is hit. */
static int recover_reexec(int usb_reset){
    const char *te=getenv("VB_TRY"); int try=(te?atoi(te):0)+1;
    if(try>10){ fprintf(stderr,"viture-bridge: giving up after %d recovery attempts - replug the Beast\n",try-1);
        return 0; }
    char tb[16]; snprintf(tb,sizeof tb,"%d",try); setenv("VB_TRY",tb,1);
    if(usb_reset){
        fprintf(stderr,"viture-bridge: USB-resetting the Beast (recovery attempt %d)...\n",try);
        usb_reset_beast();
    }
    int nap = 3 + (try<7?try:7);              /* 4..10s backoff: re-claiming too soon re-wedges */
    fprintf(stderr,"viture-bridge: settling %ds, then re-exec to recover...\n",nap);
    sleep(nap);
    execv("/proc/self/exe", g_argv);
    perror("viture-bridge: execv");           /* only reached if re-exec fails */
    return -1;
}

static int parse_qmap(const char*s,int idx[4],double sgn[4]){
    for(int i=0;i<4;i++){ while(*s==' '||*s==',')s++; double g=1;
        if(*s=='-'){g=-1;s++;} else if(*s=='+')s++;
        switch(*s){case 'w':idx[i]=0;break;case 'x':idx[i]=1;break;
                   case 'y':idx[i]=2;break;case 'z':idx[i]=3;break;default:return -1;}
        sgn[i]=g; s++; }
    return 0;
}
/* parse "z,x,y" (with optional signs) -> per-output (src axis 0..2, sign) */
static int parse_imap(const char*s,int idx[3],float sgn[3]){
    for(int i=0;i<3;i++){ while(*s==' '||*s==',')s++; float g=1;
        if(*s=='-'){g=-1;s++;} else if(*s=='+')s++;
        switch(*s){case 'x':idx[i]=0;break;case 'y':idx[i]=1;break;
                   case 'z':idx[i]=2;break;default:return -1;}
        sgn[i]=g; s++; }
    return 0;
}

/* Load live tuning overrides from /tmp (written by viture-calibrate.py or by hand),
 * applied both at startup and on SIGHUP so a calibrated map is sticky. */
static void load_tuning(void){
    char ln[64]; FILE*f;
    if((f=fopen("/tmp/viture-qmap","r"))){ if(fgets(ln,sizeof ln,f)){ ln[strcspn(ln,"\n")]=0;
        int qi[4]; double qs[4];
        if(parse_qmap(ln,qi,qs)==0){ memcpy(g_qi,qi,sizeof qi); memcpy(g_qs,qs,sizeof qs);
            fprintf(stderr,"viture-bridge: qmap -> %s\n",ln); } } fclose(f); }
    if((f=fopen("/tmp/viture-imap","r"))){ if(fgets(ln,sizeof ln,f)){ ln[strcspn(ln,"\n")]=0;
        int mi[3]; float ms[3];
        if(parse_imap(ln,mi,ms)==0){ memcpy(g_mi,mi,sizeof mi); memcpy(g_ms,ms,sizeof ms);
            g_seeded=0;  /* re-seed gravity in the new frame */
            fprintf(stderr,"viture-bridge: imu-map -> %s (re-seeding)\n",ln); } } fclose(f); }
    if((f=fopen("/tmp/viture-beta","r"))){ float b;
        if(fscanf(f,"%f",&b)==1 && b>0 && b<5){ g_ahrs.beta=b;
            fprintf(stderr,"viture-bridge: beta -> %.4f\n",b); } fclose(f); }
}

int main(int argc,char**argv){
    g_argv = argv;                /* for self-re-exec on an IMU stall */
    int port=4242, freq=2, log=1, dispmode=-1, dump_calib=0, carina=0, native=0, dofval=3; float beta=0.02f;  /* 0.02 = tremor-free on the Beast (0.08 yanked on accel) */
    const char*host="127.0.0.1",*qmap="w,x,y,z",*lib=NULL;
    for(int i=1;i<argc;i++){
        if      (!strcmp(argv[i],"--port")&&i+1<argc) port=atoi(argv[++i]);
        else if (!strcmp(argv[i],"--host")&&i+1<argc) host=argv[++i];
        else if (!strcmp(argv[i],"--qmap")&&i+1<argc) qmap=argv[++i];
        else if (!strcmp(argv[i],"--imu-map")&&i+1<argc){ if(parse_imap(argv[++i],g_mi,g_ms)){fprintf(stderr,"bad --imu-map\n");return 2;} }
        else if (!strcmp(argv[i],"--lib") &&i+1<argc) lib=argv[++i];
        else if (!strcmp(argv[i],"--freq")&&i+1<argc) freq=atoi(argv[++i]);
        else if (!strcmp(argv[i],"--display-mode")&&i+1<argc) dispmode=(int)strtol(argv[++i],0,0);
        else if (!strcmp(argv[i],"--beta")&&i+1<argc) beta=atof(argv[++i]);
        else if (!strcmp(argv[i],"--gyro-scale")&&i+1<argc) g_gyro_scale=atof(argv[++i]);
        else if (!strcmp(argv[i],"--mag")) g_use_mag=1;
        else if (!strcmp(argv[i],"--madgwick")) g_use_vqf=0;  /* A/B: use the old Madgwick filter */
        else if (!strcmp(argv[i],"--sdk-log")) log=3;
        else if (!strcmp(argv[i],"--probe-gl")) g_probe_gl=1;  /* also poll get_gl_pose (VIO probe) */
        else if (!strcmp(argv[i],"--dump-calib")) dump_calib=1; /* fetch factory cam/IMU cal blob -> /tmp/viture-calib.bin, then exit */
        else if (!strcmp(argv[i],"--carina")) carina=1;         /* XRLinuxDriver high-level 6DoF path probe */
        else if (!strcmp(argv[i],"--native")) native=1;         /* read the firmware's native fused 3DoF (anchor-mode) pose */
        else if (!strcmp(argv[i],"--dof")&&i+1<argc) dofval=atoi(argv[++i]); /* native_dof value to enable (default 3) */
        else if (!strcmp(argv[i],"--verbose")||!strcmp(argv[i],"-v")) g_verbose=1;
        else { fprintf(stderr,"usage: %s [--lib DIR] [--port N] [--host IP] [--qmap w,x,y,z] "
               "[--freq 0..4] [--beta F] [--gyro-scale F] [--mag] [--sdk-log] [--probe-gl] [--dump-calib] [-v]\n",argv[0]); return 2; }
    }
    if (parse_qmap(qmap,g_qi,g_qs)){ fprintf(stderr,"bad --qmap '%s'\n",qmap); return 2; }
    { const float hz[5]={60,90,120,240,500}; g_dt = 1.0f / hz[(freq>=0&&freq<=4)?freq:2]; }
    if (g_use_vqf){ g_vqf = vqf_create((double)g_dt);
        fprintf(stderr,"viture-bridge: VQF filter @ %.0f Hz, %s\n", 1.0/g_dt, g_use_mag?"9-axis (mag)":"6-axis"); }
    signal(SIGINT,on_sigint); signal(SIGTERM,on_sigint); signal(SIGHUP,on_hup);

    /* locate libglasses.so: --lib, $VITURE_SDK, then common spots */
    const char*env=getenv("VITURE_SDK");
    char path[1024]; void*h=NULL;
    const char*dirs[]={ lib, env, ".", "./viture-sdk", "/usr/local/lib/viture", "/usr/lib/viture" };
    for (size_t i=0;i<sizeof dirs/sizeof*dirs;i++){
        if(!dirs[i]) continue;
        snprintf(path,sizeof path,"%s/libglasses.so",dirs[i]);
        h=dlopen(path,RTLD_NOW|RTLD_GLOBAL);
        if(h){ if(g_verbose) fprintf(stderr,"viture-bridge: loaded %s\n",path); break; }
    }
    if(!h) h=dlopen("libglasses.so",RTLD_NOW|RTLD_GLOBAL);
    if(!h){ fprintf(stderr,"viture-bridge: cannot load libglasses.so (%s).\n"
            "Get the VITURE v2.0.0 aarch64 SDK (lib/aarch64/viture from wheaney/XRLinuxDriver)\n"
            "and point --lib or $VITURE_SDK at the directory holding it.\n",dlerror()); return 1; }

    xr_create=dlsym(h,"xr_device_provider_create");
    xr_init  =dlsym(h,"xr_device_provider_initialize");
    xr_start =dlsym(h,"xr_device_provider_start");
    xr_reg_raw=dlsym(h,"register_raw_callback");
    xr_reg_pose=dlsym(h,"register_pose_callback");
    xr_get_gl_pose=dlsym(h,"get_gl_pose_carina");
    xr_open_imu=dlsym(h,"open_imu");
    xr_set_dof=dlsym(h,"xr_device_provider_set_display_mode_and_native_dof");
    xr_get_dof=dlsym(h,"xr_device_provider_get_display_mode_and_native_dof");
    xr_valid =dlsym(h,"xr_device_provider_is_product_id_valid");
    xr_setlog=dlsym(h,"xr_device_provider_set_log_level");
    xr_stop  =dlsym(h,"xr_device_provider_stop");
    xr_shutdown=dlsym(h,"xr_device_provider_shutdown");
    xr_destroy=dlsym(h,"xr_device_provider_destroy");
    xr_set_display_mode=dlsym(h,"xr_device_provider_set_display_mode");
    xr_get_display_mode=dlsym(h,"xr_device_provider_get_display_mode");
    ca_imu_pose=dlsym(h,"carina_a1088_viture_get_imu_pose");
    ca_gl_pose =dlsym(h,"carina_a1088_viture_get_gl_pose");
    xr_reg_carina=dlsym(h,"register_callbacks_carina");
    xr_get_devtype=dlsym(h,"xr_device_provider_get_device_type");
    xr_reset_carina=dlsym(h,"reset_pose_carina");
    if(!xr_create||!xr_init||!xr_start||!xr_reg_raw||!xr_open_imu||!xr_set_dof){
        fprintf(stderr,"viture-bridge: SDK missing expected symbols\n"); return 1; }
    if(xr_setlog) xr_setlog(log);

    detach_cdc_acm();

    int sock=socket(AF_INET,SOCK_DGRAM,0); if(sock<0){perror("socket");return 1;}
    g_sock=sock; g_dst.sin_family=AF_INET; g_dst.sin_port=htons((uint16_t)port);
    if(inet_pton(AF_INET,host,&g_dst.sin_addr)!=1){fprintf(stderr,"bad host\n");return 1;}
    rayneo_ahrs_init(&g_ahrs,beta);

    if(xr_valid && !xr_valid(BEAST_PID))
        fprintf(stderr,"viture-bridge: WARNING SDK reports 0x%04x invalid\n",BEAST_PID);
    XRH p=xr_create(BEAST_PID);
    if(!p){ fprintf(stderr,"viture-bridge: create() failed - glasses not found / USB busy. "
            "Run with sudo (libusb needs /dev/bus/usb) and ensure cdc_acm is unbound.\n");
            recover_reexec(1);   /* cold boot: USB may not be enumerated yet - reset + retry */
            return 1; }          /* only here if execv failed or the retry cap was hit */
    xr_reg_raw(p,on_raw);
    if(xr_reg_pose){ xr_reg_pose(on_pose); fprintf(stderr,"viture-bridge: pose callback registered (probe)\n"); }
    else fprintf(stderr,"viture-bridge: no register_pose_callback in SDK\n");
    if(xr_init(p,NULL)!=0){ fprintf(stderr,"viture-bridge: initialize failed\n"); return 1; }

    /* --dump-calib: pull the Beast's factory calibration blob (camera intrinsics +
     * distortion + IMU/mag cal + cam-display optical params) over USB and write it
     * raw to /tmp/viture-calib.bin, then exit. This is the input the Carina VIO
     * config.yaml needs (fisheye intrinsics + T_cam_imu). Done BEFORE open_imu/start
     * so the calibration control read doesn't contend with the IMU stream.
     * get_calibration_config(handle, buf, &len): *len must be set to buf capacity on
     * entry; returns 0 ok (len=actual), -1 none, -2 buf<actual (max 256KB), -3 type. */
    if(dump_calib){
        int (*get_calib)(void*,unsigned char*,int*) =
            dlsym(h,"_ZN6viture8internal22get_calibration_configEPvPhPi");
        int (*close_imu_fn)(void*) = dlsym(h,"close_imu");
        if(!get_calib){ fprintf(stderr,"viture-bridge: get_calibration_config missing in SDK\n"); }
        else {
            /* The calibration blob comes back as a multi-segment USB long-packet on
             * the SAME CDC-ACM pipe that streams pose/IMU. Any live stream interleaves
             * into the response and reassembly fails CRC. Two streams to kill first:
             *  (1) the Beast's NATIVE on-glasses 3DOF pose (on by default) - disable it
             *      via set_dof(...,OFF), exactly like the normal bridge does; and
             *  (2) raw IMU, via close_imu. Then drain the pipe before reading. */
            { int dm0=0x36,nd0=0; if(xr_get_dof) xr_get_dof(p,&dm0,&nd0); if(dm0<0) dm0=0x36;
              int sr=xr_set_dof(p,dm0,NATIVE_DOF_OFF);
              fprintf(stderr,"viture-bridge: set_dof(0x%x,OFF) -> %d (disabling native 3DOF stream)\n",dm0,sr); }
            if(close_imu_fn){ int cr=close_imu_fn(p);
                fprintf(stderr,"viture-bridge: close_imu -> %d (quieting pipe for calib read)\n",cr); }
            struct timespec drain={0,800000000}; nanosleep(&drain,NULL);   /* 800ms drain */
            int r=-1, len=0; static unsigned char buf[262144];
            for(int attempt=1; attempt<=5; attempt++){
                len=(int)sizeof buf;
                r=get_calib(p,buf,&len);
                fprintf(stderr,"viture-bridge: get_calibration_config attempt %d -> r=%d len=%d\n",attempt,r,len);
                if(r==0 && len>0) break;
                struct timespec rt={0,300000000}; nanosleep(&rt,NULL);
            }
            if(r==0 && len>0){
                FILE*cf=fopen("/tmp/viture-calib.bin","wb");
                if(cf){ size_t w=fwrite(buf,1,(size_t)len,cf); fclose(cf);
                    fprintf(stderr,"viture-bridge: wrote /tmp/viture-calib.bin (%zu bytes)\n",w); }
                else perror("viture-bridge: open /tmp/viture-calib.bin");
            } else fprintf(stderr,"viture-bridge: calibration read failed after retries (r=%d)\n",r);
        }
        if(xr_stop) xr_stop(p);
        if(xr_shutdown) xr_shutdown(p);
        if(xr_destroy) xr_destroy(p);
        close(sock);
        return 0;
    }

    /* --carina: the XRLinuxDriver high-level 6DoF path. NO open_imu / set_dof / config -
     * the SDK's CarinaDeviceProvider reads factory cal + drives cameras + runs the VIO
     * internally. The whole question: does the Beast report device_type==CARINA so this
     * is even available? register_callbacks_carina returns -1 if the provider isn't a
     * CarinaDeviceProvider. If poses stream (with translation in [0..2]), Beast does 6DoF. */
    if(carina){
        int dt = xr_get_devtype ? xr_get_devtype(p) : -99;
        fprintf(stderr,"viture-bridge: xr_device_provider_get_device_type -> %d\n", dt);
        if(!xr_reg_carina){ fprintf(stderr,"viture-bridge: register_callbacks_carina missing in SDK\n"); }
        else {
            int rr = xr_reg_carina(p, on_carina_pose, NULL, NULL, NULL);
            fprintf(stderr,"viture-bridge: register_callbacks_carina -> %d %s\n", rr,
                    rr==0?"(OK - Beast IS carina-capable!)":"(FAILED - not a Carina provider)");
        }
        xr_start(p);
        fprintf(stderr,"viture-bridge: carina started; polling get_gl_pose_carina + pose callback...\n");
        if(xr_reset_carina){ xr_reset_carina(p); fprintf(stderr,"viture-bridge: reset_pose_carina called (anchor here)\n"); }
        double l=0;
        while(g_run){
            struct timespec ts={0,100000000}; nanosleep(&ts,NULL);
            double t=now_sec();
            if(xr_get_gl_pose && t-l>=0.3){ l=t; float gp[16]={0}; int r=xr_get_gl_pose(p,gp,0.0);
                fprintf(stderr,"GLPOSE r=%d | pos % .4f % .4f % .4f | rot % .4f % .4f % .4f % .4f\n",
                        r, gp[0],gp[1],gp[2], gp[3],gp[4],gp[5],gp[6]); }
        }
        if(xr_stop) xr_stop(p); if(xr_shutdown) xr_shutdown(p); if(xr_destroy) xr_destroy(p);
        close(sock); return 0;
    }

    /* --native: read the FIRMWARE's native fused 3DoF pose (what "anchor mode" uses) -
     * the opposite of our normal path. We do NOT disable native DOF and do NOT open_imu
     * RAW; instead we ENABLE native 3DoF and let register_pose_callback (on_pose, already
     * registered) deliver the factory-calibrated, firmware-fused orientation. This is
     * XRLinuxDriver's legacy (non-carina) path for device_type 1 like the Beast. */
    if(native){
        /* XRLinuxDriver legacy recipe EXACTLY: NO set_dof (don't disable native fusion,
         * and don't touch the display mode - safer too), start, THEN open_imu(POSE).
         * The earlier "open_imu(POSE)=-2" was because we'd called set_dof(OFF) first. */
        (void)dofval;
        fprintf(stderr,"viture-bridge: native: sleep(1) -> start -> open_imu(POSE)...\n");
        sleep(1);
        xr_start(p);
        int oi = xr_open_imu(p, IMU_MODE_POSE, (uint8_t)freq);
        fprintf(stderr,"viture-bridge: open_imu(POSE=%d, freq=%d) -> %d %s\n",
                IMU_MODE_POSE, freq, oi, oi==0?"OK":"(FAILED)");
        fprintf(stderr,"viture-bridge: waiting for SDK fused-pose callback (rotate the glasses)...\n");
        int waited=0;
        while(g_run){
            struct timespec ts={0,200000000}; nanosleep(&ts,NULL);
            if(g_pose_n==0 && ++waited%5==0) fprintf(stderr,"viture-bridge: no native pose yet (%d)...\n",waited);
        }
        if(xr_stop) xr_stop(p); if(xr_shutdown) xr_shutdown(p); if(xr_destroy) xr_destroy(p);
        close(sock); return 0;
    }

    int dm=0x36,nd=0;
    if(xr_get_dof) xr_get_dof(p,&dm,&nd);
    if(dm<0) dm=0x36;
    /* optional: switch the panel to a 120Hz mode (the Beast defaults its DP link
     * to 60Hz; e.g. 0x34=1920x1080@120, 0x44=1920x1200@120). */
    if(dispmode>=0 && xr_set_display_mode){
        int before = xr_get_display_mode ? xr_get_display_mode(p) : -99;
        int r=xr_set_display_mode(p,dispmode);
        struct timespec ms={0,300000000}; nanosleep(&ms,NULL);
        int after = xr_get_display_mode ? xr_get_display_mode(p) : -99;
        fprintf(stderr,"viture-bridge: display_mode was 0x%x, set(0x%02x)->%d, now 0x%x\n",
                before,dispmode,r,after);
        dm=dispmode;
    }
    xr_set_dof(p,dm,NATIVE_DOF_OFF);                 /* disable on-glasses 3DOF */
    int imu_rc = xr_open_imu(p,IMU_MODE_RAW,(uint8_t)freq);
    if(imu_rc!=0)
        fprintf(stderr,"viture-bridge: open_imu(RAW) failed (rc=%d)\n",imu_rc);
    xr_start(p);
    double t_start = now_sec();                       /* cold-start watchdog baseline */
    /* If open_imu already errored the control pipe is wedged - recover fast (2s)
     * instead of waiting out the full cold timeout. */
    double cold_to = (imu_rc!=0) ? 2.0 : 5.0;
    fprintf(stderr,"viture-bridge: Beast streaming -> %s:%d (qmap=%s, %s)\n",
            host,port,qmap,g_use_mag?"9-axis":"6-axis");

    load_tuning();   /* apply any /tmp/viture-{imap,qmap,beta} now so a calibrated
                        map survives bridge restarts AND watchdog re-execs (the
                        callback only re-reads them on SIGHUP otherwise). */

    if(g_probe_gl)
        fprintf(stderr,"viture-bridge: --probe-gl ON | get_gl_pose_carina=%s "
                "carina_get_imu_pose=%s carina_get_gl_pose=%s\n",
                xr_get_gl_pose?"yes":"NO", ca_imu_pose?"yes":"NO", ca_gl_pose?"yes":"NO");

    while(g_run) { struct timespec t={0,200000000}; nanosleep(&t,NULL);
        if(g_reload){ g_reload=0; load_tuning();
        }
        if(g_verbose && g_n==0){ static int w=0; if(++w%5==0) fprintf(stderr,"viture-bridge: no IMU yet...\n"); }

        /* Once samples flow, clear the cross-re-exec attempt counter so a later
         * (warm) stall recovery starts its backoff from scratch. */
        if(g_n>0 && getenv("VB_TRY")) unsetenv("VB_TRY");

        /* Cold-start watchdog: open_imu can fail on a fresh boot ("Failed to send
         * USB command -1"), so NO sample ever arrives and the stall watchdog below
         * (which needs g_n>0) can never fire. Catch the never-streamed case and
         * recover via a USB reset + re-exec - no physical replug needed. */
        if(g_n==0 && now_sec()-t_start > cold_to){
            fprintf(stderr,"viture-bridge: no IMU %.0fs after cold start - USB wedged, recovering\n",cold_to);
            g_cold=1; g_stall=1; break;
        }

        /* VIO probe: poll the carina GL pose. If it returns 0 with a 6DoF transform
         * (translation in [4..6] that tracks as you lean), the SDK is doing VIO; if
         * it errors or is static, it isn't (for the Beast over this path). */
        if(g_probe_gl && now_sec()-g_capose_log >= 0.5){
            g_capose_log = now_sec();
            /* libglasses wrapper: get_gl_pose_carina(handle, out, ts) */
            if(xr_get_gl_pose){
                float gp[16]={0}; int r=xr_get_gl_pose(p,gp,0.0);
                fprintf(stderr,"GLPOSE  r=%d | % .4f % .4f % .4f % .4f | % .4f % .4f % .4f\n",
                        r, gp[0],gp[1],gp[2],gp[3], gp[4],gp[5],gp[6]);
            }
            /* Tier-1: VITURE's own IMU fusion (the rock-solid anchor, no camera) */
            if(ca_imu_pose){
                float ip[16]={0}; int r=ca_imu_pose(ip,0.0);
                fprintf(stderr,"IMUPOSE r=%d | % .4f % .4f % .4f % .4f | % .4f % .4f % .4f\n",
                        r, ip[0],ip[1],ip[2],ip[3], ip[4],ip[5],ip[6]);
            }
            /* Tier-2: a1088 6DOF GL transform (live only if the VIO path is running) */
            if(ca_gl_pose){
                float gp[16]={0}; int r=ca_gl_pose(gp,0.0);
                fprintf(stderr,"A1088GL r=%d | % .4f % .4f % .4f % .4f | % .4f % .4f % .4f\n",
                        r, gp[0],gp[1],gp[2],gp[3], gp[4],gp[5],gp[6]);
            }
        }

        /* Stall watchdog. The Beast is a combined DP+USB device: when the display
         * link drops/re-modes (apple-dcp dcp_dptx_disconnect), the SDK's USB read
         * silently stops - the process stays alive but no more raw callbacks fire,
         * so mirage sees pose 0 Hz. There's no SDK signal for this, so detect it by
         * the sample clock going quiet and break out; the cleanup path then re-execs
         * us (same PID), which re-detaches cdc_acm and re-claims the IMU in ~2s. */
        if(g_n>0 && g_last_raw>0 && now_sec()-g_last_raw > 1.5){
            fprintf(stderr,"viture-bridge: IMU stall (no samples for >1.5s) after %ld - recovering\n",g_n);
            g_stall=1; break;
        }
    }

    fprintf(stderr,"\nviture-bridge: stopping (%ld samples)\n",g_n);
    /* clean release so the next run doesn't need a replug: stop -> shutdown ->
     * destroy lets the SDK release the libusb interface it claimed. */
    if (xr_stop) xr_stop(p);
    if (xr_shutdown) xr_shutdown(p);
    if (xr_destroy) xr_destroy(p);
    close(sock);

    /* Stall recovery: re-exec ourselves (same PID + name, so pgrep/pkill -x still
     * work and a clean SIGTERM stays down). The SDK released the USB interface in
     * the shutdown above; a fresh start re-detaches cdc_acm and re-claims the IMU.
     * A genuine SIGINT/SIGTERM clears g_run via the handler but NOT g_stall, so we
     * only loop back on a watchdog trip. */
    if (g_stall && g_run){
        /* g_cold (never streamed) means the USB control pipe is wedged, not just a
         * mid-stream DP blip - escalate to a USB-level reset (replug-equivalent).
         * A warm stall released its claim cleanly in the shutdown above, so it only
         * needs the settle + re-exec. recover_reexec backs off, bumps the attempt
         * counter and re-execs; it only returns if execv fails or the cap is hit. */
        recover_reexec(g_cold);
        return 3;
    }
    return 0;
}
