#ifndef EEZ_LVGL_UI_STYLES_H
#define EEZ_LVGL_UI_STYLES_H

#include <../../lvgl/lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Style: number_display
lv_style_t *get_style_number_display_MAIN_DEFAULT();
void add_style_number_display(lv_obj_t *obj);
void remove_style_number_display(lv_obj_t *obj);



#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_STYLES_H*/