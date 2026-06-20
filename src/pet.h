/* pet.h - a small mischievous reactive pet that lives in the head-tracked space.
 *
 * Fully self-contained: its own GL program, sphere mesh, and behaviour state
 * machine all live in pet.cpp. The renderer touches it in exactly three places:
 *
 *   render_init   -> pet_init(m)                       (once, GL context current)
 *   render_frame  -> pet_draw(m, vp, eye_world, head)  (per frame, after screens)
 *   render_finish -> pet_finish()                      (once, context still live)
 *
 * The pet reads head pose to decide whether you're looking at it, and behaves
 * accordingly: naps at a home spot, gets bored and prowls off to "mess with" a
 * screen, freezes guiltily the instant your gaze swings onto it, then flees home.
 * This Tier-0 build is procedural (a blob with googly tracking eyes) - no external
 * assets. The screen-nudging (a reversible per-screen offset layer) is the next
 * step and is deliberately NOT wired in yet, to keep this isolated.
 */
#ifndef MIRAGE_PET_H
#define MIRAGE_PET_H

#include "math3d.h"

struct mirage;

void pet_init(struct mirage *m);
/* vp = proj*view, eye_world = viewer position (world), head = view orientation.
 * These are exactly the values render_frame already has on hand. */
void pet_draw(struct mirage *m, mat4 vp, vec3 eye_world, quat head);
void pet_finish(void);

#endif /* MIRAGE_PET_H */
