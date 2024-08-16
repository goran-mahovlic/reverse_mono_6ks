#ifndef EEZ_LVGL_UI_EVENTS_H
#define EEZ_LVGL_UI_EVENTS_H

#include <../../lvgl/lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void action_move_up(lv_event_t * e);
extern void action_move_down(lv_event_t * e);
extern void action_home_z(lv_event_t * e);
extern void action_middle_z(lv_event_t * e);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_EVENTS_H*/