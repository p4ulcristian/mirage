/* camera.h - minimal UVC (V4L2 + MJPEG) capture for camera passthrough.
 *
 * Opens a USB camera, streams MJPEG on a background thread, decodes each frame to
 * RGB888 (libturbojpeg) into a triple buffer, and hands the newest frame to the
 * render thread lock-free-ish (one short mutex per frame, no copy on acquire).
 *
 * On the VITURE Beast the world-facing camera is a plain UVC device on the glasses'
 * USB-C hub (Sonix 0c45:6368), so this needs no SDK - just /dev/videoN.
 */
#pragma once
#include <cstdint>

struct cam;                                  /* opaque */

/* Start streaming dev at WxH (MJPEG). Returns nullptr on any failure (logs why). */
cam *cam_start(const char *dev, int w, int h);

/* Stop the thread and release the device. */
void cam_stop(cam *c);

/* If a frame newer than *seq exists, point *rgb at it (RGB888, w*h*3 bytes, valid
 * until the next cam_acquire), update w/h/seq, and return true; else false.
 * Pass *seq = the value from the previous successful call (start at 0). */
bool cam_acquire(cam *c, const uint8_t **rgb, int *w, int *h, uint64_t *seq);
