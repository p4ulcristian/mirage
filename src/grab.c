#include "mirage.h"

/* Input grab (Super+G) is implemented in task 6 using
 * hyprland_global_shortcuts_manager_v1 + zwlr_virtual_pointer_manager_v1.
 * Stubbed for now so the renderer can stand on its own. */
bool grab_init(struct mirage *m)  { (void)m; return true; }
void grab_destroy(struct mirage *m) { (void)m; }
