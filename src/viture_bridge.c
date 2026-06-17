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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "rayneo.h"   /* reuse the Madgwick AHRS (rayneo_ahrs_*) + rayneo_imu */

#define VITURE_VID  0x35ca
#define BEAST_PID   0x1201

/* ---- VITURE v2.0.0 SDK ABI (from XRLinuxDriver's viture headers) ---- */
typedef void* XRH;
typedef void (*raw_cb)(float *data, uint64_t ts, uint64_t vsync);
/* IMU raw layout (Beast/Luma): gyro xyz, accel xyz, mag xyz, temp (10 floats) */
#define IMU_MODE_RAW   0
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

static volatile sig_atomic_t g_run = 1;
static int g_stall = 0;           /* set when the watchdog trips -> re-exec to recover */
static char **g_argv = NULL;      /* saved for self-re-exec on stall */
static void on_sigint(int s){ (void)s; g_run = 0; }
static volatile sig_atomic_t g_reload = 0;
static void on_hup(int s){ (void)s; g_reload = 1; }  /* re-read /tmp/viture-qmap live */
static double now_sec(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);
    return t.tv_sec + t.tv_nsec*1e-9; }

/* ---- shared with the SDK's IMU callback thread ---- */
static rayneo_ahrs g_ahrs;
static int g_sock; static struct sockaddr_in g_dst;
static int g_qi[4]; static double g_qs[4];      /* output quaternion remap */
/* input axis remap, Beast sensor->AHRS frame. Default = "-z,x,-y" (forward<-(-z),
 * left-right<-(+x), up<-(-y)): verified on the Beast so head motion tracks 1:1 with
 * mirage's RayNeo-tuned device->world remap. Derive afresh with viture-calibrate.py. */
static int g_mi[3]={2,0,1}; static float g_ms[3]={-1,1,-1};
static double g_gyro_scale = 1.0;                /* Beast gyro is already rad/s (peak ~7 = ~420 deg/s) */
static float g_dt = 1.0f/120.0f;                 /* fixed sample dt (SDK streams at a set rate) - avoids wall-clock jitter that shows as tremor */
static int g_verbose, g_use_mag;
static int g_seeded = 0;
static float g_gyro_bias[3] = {0,0,0};
static long g_n = 0;
static double g_last_log = 0;
static double g_last_raw = 0;     /* monotonic time of the last raw IMU callback (stall watchdog) */

static void on_raw(float *d, uint64_t ts, uint64_t vsync){
    (void)ts; (void)vsync;
    /* fixed dt from the SDK's streaming rate - wall-clock dt in this callback
     * jitters badly (batched delivery) and that integration jitter is the tremor. */
    float dt = g_dt;

    /* input axis remap (Beast sensor frame -> AHRS frame): apply the same
     * permutation/signs to gyro, accel and mag so the fused frame is consistent. */
    rayneo_imu s = {0};
    for (int i=0;i<3;i++){
        int j=g_mi[i]; float sg=g_ms[i];
        s.gyro_rad[i] = sg * d[j]    * (float)g_gyro_scale;
        s.accel[i]    = sg * d[3+j];
        s.mag[i]      = sg * d[6+j];
    }

    /* seed orientation to gravity once (pitch/roll start level) */
    if (!g_seeded){
        rayneo_imu g = s; g.gyro_rad[0]=g.gyro_rad[1]=g.gyro_rad[2]=0;
        float sb=g_ahrs.beta; g_ahrs.beta=2.0f;
        for (int it=0; it<400; it++) rayneo_ahrs_update(&g_ahrs,&g,0.01f);
        g_ahrs.beta=sb; g_seeded=1;
    }
    /* gyro bias auto-zero while still (kills heading drift) */
    float gm = sqrtf(s.gyro_rad[0]*s.gyro_rad[0]+s.gyro_rad[1]*s.gyro_rad[1]+s.gyro_rad[2]*s.gyro_rad[2]);
    if (gm < 2.5f*(float)(M_PI/180.0)){
        float lr = dt/1.5f; if (lr>0.05f) lr=0.05f;
        for (int k=0;k<3;k++) g_gyro_bias[k]+=lr*(s.gyro_rad[k]-g_gyro_bias[k]);
    }
    for (int k=0;k<3;k++) s.gyro_rad[k]-=g_gyro_bias[k];

    if (g_use_mag) rayneo_ahrs_update9(&g_ahrs,&s,s.mag,dt);
    else           rayneo_ahrs_update(&g_ahrs,&s,dt);

    double src[4] = { g_ahrs.q[0], g_ahrs.q[1], g_ahrs.q[2], g_ahrs.q[3] };
    double out[4]; for (int k=0;k<4;k++) out[k]=g_qs[k]*src[g_qi[k]];
    double nm = sqrt(out[0]*out[0]+out[1]*out[1]+out[2]*out[2]+out[3]*out[3]);
    if (nm>1e-9) for(int k=0;k<4;k++) out[k]/=nm;
    sendto(g_sock,out,sizeof out,0,(struct sockaddr*)&g_dst,sizeof g_dst);
    g_n++;

    double t = now_sec();
    g_last_raw = t;               /* feed the stall watchdog */
    if (g_verbose && t-g_last_log>=0.25){
        float yaw,pitch,roll; rayneo_ahrs_euler(&g_ahrs,&yaw,&pitch,&roll);
        fprintf(stderr,"ypr % 7.1f % 7.1f % 7.1f | RAWacc % 6.2f % 6.2f % 6.2f | RAWgyr % 6.2f % 6.2f % 6.2f | n %ld\n",
                yaw,pitch,roll, d[3],d[4],d[5], d[0],d[1],d[2], g_n);
        g_last_log=t;
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
    int port=4242, freq=2, log=1, dispmode=-1; float beta=0.02f;  /* 0.02 = tremor-free on the Beast (0.08 yanked on accel) */
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
        else if (!strcmp(argv[i],"--sdk-log")) log=3;
        else if (!strcmp(argv[i],"--verbose")||!strcmp(argv[i],"-v")) g_verbose=1;
        else { fprintf(stderr,"usage: %s [--lib DIR] [--port N] [--host IP] [--qmap w,x,y,z] "
               "[--freq 0..4] [--beta F] [--gyro-scale F] [--mag] [--sdk-log] [-v]\n",argv[0]); return 2; }
    }
    if (parse_qmap(qmap,g_qi,g_qs)){ fprintf(stderr,"bad --qmap '%s'\n",qmap); return 2; }
    { const float hz[5]={60,90,120,240,500}; g_dt = 1.0f / hz[(freq>=0&&freq<=4)?freq:2]; }
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
            "Run with sudo (libusb needs /dev/bus/usb) and ensure cdc_acm is unbound.\n"); return 1; }
    xr_reg_raw(p,on_raw);
    if(xr_init(p,NULL)!=0){ fprintf(stderr,"viture-bridge: initialize failed\n"); return 1; }
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
    if(xr_open_imu(p,IMU_MODE_RAW,(uint8_t)freq)!=0)
        fprintf(stderr,"viture-bridge: open_imu(RAW) failed\n");
    xr_start(p);
    fprintf(stderr,"viture-bridge: Beast streaming -> %s:%d (qmap=%s, %s)\n",
            host,port,qmap,g_use_mag?"9-axis":"6-axis");

    load_tuning();   /* apply any /tmp/viture-{imap,qmap,beta} now so a calibrated
                        map survives bridge restarts AND watchdog re-execs (the
                        callback only re-reads them on SIGHUP otherwise). */

    while(g_run) { struct timespec t={0,200000000}; nanosleep(&t,NULL);
        if(g_reload){ g_reload=0; load_tuning();
        }
        if(g_verbose && g_n==0){ static int w=0; if(++w%5==0) fprintf(stderr,"viture-bridge: no IMU yet...\n"); }

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
        fprintf(stderr,"viture-bridge: re-exec to recover the IMU stream...\n");
        sleep(1);                                 /* let cdc_acm settle / USB quiesce */
        execv("/proc/self/exe", g_argv);
        perror("viture-bridge: execv");           /* only reached if re-exec fails */
        return 3;
    }
    return 0;
}
