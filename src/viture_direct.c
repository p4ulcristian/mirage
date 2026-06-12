/*
 * viture_direct.c - SDK-free Beast tracking via CDC-ACM serial
 *
 * The Beast (0x35CA:0x1201) exposes a CDC-ACM serial interface.
 * On Linux it shows up as /dev/ttyACM*. We just need to:
 *   1. Open the serial port
 *   2. Send enable IMU command
 *   3. Read 64-byte packets
 *   4. Parse orientation and send to mirage
 *
 * NO SDK REQUIRED.
 *
 * IMPORTANT: This is experimental. The Beast may send:
 *   - Pre-fused euler angles (what we want) - just parse and convert
 *   - Raw gyro/accel (what we don't want) - would need Madgwick/VQF fusion
 *
 * The SDK has two modes: IMU_MODE_RAW (0) and IMU_MODE_POSE (1).
 * The simple 0x15 command might give euler angles (older protocol).
 * If tracking is wobbly, it's likely raw data and needs fusion.
 *
 * Try `viture-bridge --native` first (uses SDK's POSE mode).
 * If that works, it confirms firmware fusion is available.
 * Then we can reverse-engineer the exact HID command for POSE mode.
 *
 * Usage: sudo ./viture-direct /dev/ttyACM0
 *
 * Linux-only (uses /dev/ttyACM*, /sys/class/tty).
 */

#define _POSIX_C_SOURCE 199309L
#ifdef __APPLE__
#include <mach/mach_time.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <dirent.h>

#define PACKET_SIZE 64
#define VITURE_VID  0x35ca
#define BEAST_PID   0x1201

/* Packet headers */
#define HDR_IMU_0   0xFF
#define HDR_IMU_1   0xFC
#define HDR_MCU_0   0xFF
#define HDR_MCU_1   0xFE

/* Commands */
#define CMD_SET_IMU 0x15

static volatile sig_atomic_t g_run = 1;
static void on_sig(int s) { (void)s; g_run = 0; }

/* CRC-16-CCITT (poly 0x1021, init 0xFFFF) */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

/* Build a 64-byte MCU command packet */
static void build_mcu_cmd(uint8_t *pkt, uint16_t cmd_id, uint8_t data_byte) {
    memset(pkt, 0, PACKET_SIZE);
    pkt[0] = HDR_MCU_0;
    pkt[1] = HDR_MCU_1;
    /* payload_len at offset 4-5 (little-endian) */
    uint16_t payload_len = 12;  /* minimal payload */
    pkt[4] = payload_len & 0xFF;
    pkt[5] = (payload_len >> 8) & 0xFF;
    /* cmd_id at offset 14-15 */
    pkt[14] = cmd_id & 0xFF;
    pkt[15] = (cmd_id >> 8) & 0xFF;
    /* data at offset 18 */
    pkt[18] = data_byte;
    /* CRC over bytes 4+ */
    uint16_t crc = crc16_ccitt(pkt + 4, PACKET_SIZE - 4);
    pkt[2] = crc & 0xFF;
    pkt[3] = (crc >> 8) & 0xFF;
}

/* Euler (degrees) to quaternion (w,x,y,z) */
static void euler_to_quat(float roll, float pitch, float yaw, double *q) {
    double r = roll * M_PI / 180.0 / 2.0;
    double p = pitch * M_PI / 180.0 / 2.0;
    double y = yaw * M_PI / 180.0 / 2.0;
    double cr = cos(r), sr = sin(r);
    double cp = cos(p), sp = sin(p);
    double cy = cos(y), sy = sin(y);
    q[0] = cr*cp*cy + sr*sp*sy;  /* w */
    q[1] = sr*cp*cy - cr*sp*sy;  /* x */
    q[2] = cr*sp*cy + sr*cp*sy;  /* y */
    q[3] = cr*cp*sy - sr*sp*cy;  /* z */
}

/* Find Beast serial port by scanning /sys/class/tty */
static char *find_beast_tty(void) {
    static char path[256];
    DIR *d = opendir("/sys/class/tty");
    if (!d) return NULL;
    struct dirent *e;
    while ((e = readdir(d))) {
        if (strncmp(e->d_name, "ttyACM", 6) != 0) continue;
        char uevent[512];
        snprintf(uevent, sizeof(uevent), "/sys/class/tty/%s/device/../uevent", e->d_name);
        FILE *f = fopen(uevent, "r");
        if (!f) continue;
        char line[256];
        int found = 0;
        while (fgets(line, sizeof(line), f)) {
            /* Look for PRODUCT=35ca/1201/... */
            if (strstr(line, "PRODUCT=35ca/1201") || strstr(line, "PRODUCT=35CA/1201")) {
                found = 1;
                break;
            }
        }
        fclose(f);
        if (found) {
            snprintf(path, sizeof(path), "/dev/%s", e->d_name);
            closedir(d);
            return path;
        }
    }
    closedir(d);
    return NULL;
}

int main(int argc, char **argv) {
    const char *dev = argc > 1 ? argv[1] : NULL;
    const char *host = "127.0.0.1";
    int port = 4242;
    int verbose = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-v")) verbose = 1;
        else if (!strcmp(argv[i], "--port") && i+1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--host") && i+1 < argc) host = argv[++i];
        else if (argv[i][0] == '/') dev = argv[i];
    }

    if (!dev) {
        dev = find_beast_tty();
        if (!dev) {
            fprintf(stderr, "viture-direct: Beast not found. Specify device: %s /dev/ttyACM0\n", argv[0]);
            return 1;
        }
        fprintf(stderr, "viture-direct: auto-detected %s\n", dev);
    }

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    /* Open serial port */
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("open");
        fprintf(stderr, "viture-direct: cannot open %s (need sudo?)\n", dev);
        return 1;
    }

    /* Configure serial: 115200 8N1, raw mode */
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return 1;
    }
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return 1;
    }

    /* UDP socket for mirage */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); close(fd); return 1; }
    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(port);
    inet_pton(AF_INET, host, &dst.sin_addr);

    /* Send enable IMU command */
    uint8_t cmd[PACKET_SIZE];
    build_mcu_cmd(cmd, CMD_SET_IMU, 0x01);
    if (write(fd, cmd, PACKET_SIZE) != PACKET_SIZE) {
        perror("write enable cmd");
    }
    fprintf(stderr, "viture-direct: sent enable IMU command\n");

    /* Read loop */
    uint8_t buf[PACKET_SIZE];
    long n = 0;
    double last_log = 0;

    fprintf(stderr, "viture-direct: streaming to %s:%d (Ctrl-C to stop)\n", host, port);

    while (g_run) {
        /* Read one packet */
        ssize_t r = read(fd, buf, PACKET_SIZE);
        if (r <= 0) {
            if (r < 0) perror("read");
            usleep(1000);
            continue;
        }

        /* Sync to IMU header */
        if (r < PACKET_SIZE || buf[0] != HDR_IMU_0 || buf[1] != HDR_IMU_1) {
            /* Try to resync - skip bytes until we find header */
            continue;
        }

        /* Parse euler angles from data payload (offset 18) */
        float roll, pitch, yaw;
        memcpy(&roll, buf + 18, 4);
        memcpy(&pitch, buf + 18 + 4, 4);
        memcpy(&yaw, buf + 18 + 8, 4);
        yaw = -yaw;  /* negate yaw per protocol */

        /* Convert to quaternion */
        double quat[4];
        euler_to_quat(roll, pitch, yaw, quat);

        /* Send to mirage */
        double pkt[7] = { quat[0], quat[1], quat[2], quat[3], 0, 0, 0 };
        sendto(sock, pkt, sizeof(pkt), 0, (struct sockaddr*)&dst, sizeof(dst));
        n++;

        if (verbose) {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            double t = ts.tv_sec + ts.tv_nsec * 1e-9;
            if (t - last_log >= 0.25) {
                fprintf(stderr, "ypr % 7.1f % 7.1f % 7.1f | n %ld\n", yaw, pitch, roll, n);
                last_log = t;
            }
        }
    }

    /* Disable IMU */
    build_mcu_cmd(cmd, CMD_SET_IMU, 0x00);
    write(fd, cmd, PACKET_SIZE);

    fprintf(stderr, "\nviture-direct: stopped (%ld samples)\n", n);
    close(sock);
    close(fd);
    return 0;
}
