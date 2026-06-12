/* entity.hpp - things that live in the curved space.
 *
 * The wall is becoming a scene of ENTITIES, not just captured windows. Each entity
 * has a place on the cylinder and a way to draw itself; later it'll also own how it
 * responds to the cursor (Display forwards to the app, a Model orbits, a Banner
 * fires an action). This is the first slice: Banner - a baked text/graphics panel
 * hung in the same space as the displays. The clock is the first Banner; status
 * lines, labels, and later Model / Webapp entities join the same scene.
 *
 * (Displays are still screen_t for now; they fold into this scene in a later step,
 * at which point the cursor hit-tests entities generically and dispatches by kind.)
 */
#ifndef MIRAGE_ENTITY_HPP
#define MIRAGE_ENTITY_HPP

#include "handle.hpp"

#include <functional>
#include <string>

namespace ent {

/* A flat panel of baked text/graphics on a curved strip, placed on the cylinder. */
struct Banner {
    /* placement */
    float yaw  = 0.0f;        /* radians, +yaw = viewer's left */
    float lift = 0.0f;        /* metres above eye level        */
    float arc  = 30.0f;       /* angular width (degrees)       */
    float color[3] = {1.0f, 1.0f, 1.0f};

    /* content: text() produces the string (may contain '\n'); key() changes when
     * the content should be re-baked (e.g. the second-of-day for a clock). */
    std::function<std::string()> text;
    std::function<int()>         key;

    /* runtime, built/owned by the renderer */
    own::GlBuffer  vbo;
    int            verts    = 0;
    own::GlTexture tex;
    int            tw = 0, th = 0;
    int            last_key = -1;
};

} // namespace ent

#endif /* MIRAGE_ENTITY_HPP */
