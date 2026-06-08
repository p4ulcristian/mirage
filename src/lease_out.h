/* lease_out.h - leased-KMS output backend for mirage on the glasses.
 *
 * On Asahi the glasses are flagged non-desktop (kernel apple-dcp patch), so
 * Hyprland withdraws DP-1 from the desktop and offers it via wp_drm_lease. mirage
 * leases it and scans out DIRECTLY: GBM buffers + EGL + atomic page-flips, no
 * Hyprland compositing involved at all - so no bar/cursor/scanout dance, locked
 * 120Hz by construction. render.c is unchanged; only how the final frame reaches
 * the panel differs from the Wayland-surface path.
 */
#ifndef MIRAGE_LEASE_OUT_H
#define MIRAGE_LEASE_OUT_H
#include <stdbool.h>
struct mirage;

/* Lease DP-1 from the compositor, set up GBM + EGL (fills m->edpy/ecfg/ectx/esurf,
 * makes the context current) and m->glasses_w/h from the panel mode. Returns false
 * if the connector isn't offered for lease (kernel patch missing) or KMS setup
 * fails - caller then falls back to the Wayland-surface path. */
bool lease_out_init(struct mirage *m);

/* Present the rendered frame: eglSwapBuffers + atomic page-flip, then block on the
 * flip completion (this is the vsync that paces the loop at the panel's refresh). */
void lease_out_present(struct mirage *m);

/* Release the lease (the compositor reclaims DP-1) and tear down GBM/KMS. */
void lease_out_finish(struct mirage *m);

#endif /* MIRAGE_LEASE_OUT_H */
