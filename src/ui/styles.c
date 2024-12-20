#include "styles.h"
#include "images.h"
#include "fonts.h"

#include "screens.h"

//
// Style: number_display
//

void init_style_number_display_MAIN_DEFAULT(lv_style_t *style) {
    lv_style_set_pad_top(style, 3);
    lv_style_set_pad_bottom(style, 1);
    lv_style_set_pad_left(style, 1);
    lv_style_set_pad_right(style, 1);
    lv_style_set_bg_color(style, lv_color_hex(0xffffffff));
    lv_style_set_bg_opa(style, 255);
    lv_style_set_border_color(style, lv_color_hex(0xff333333));
    lv_style_set_text_color(style, lv_color_hex(0xff0aacf5));
    lv_style_set_text_font(style, &ui_font_robot);
    lv_style_set_text_align(style, LV_TEXT_ALIGN_CENTER);
};

lv_style_t *get_style_number_display_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = lv_mem_alloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_number_display_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_number_display(lv_obj_t *obj) {
    lv_obj_add_style(obj, get_style_number_display_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_number_display(lv_obj_t *obj) {
    lv_obj_remove_style(obj, get_style_number_display_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
//
//

void add_style(lv_obj_t *obj, int32_t styleIndex) {
    typedef void (*AddStyleFunc)(lv_obj_t *obj);
    static const AddStyleFunc add_style_funcs[] = {
        add_style_number_display,
    };
    add_style_funcs[styleIndex](obj);
}

void remove_style(lv_obj_t *obj, int32_t styleIndex) {
    typedef void (*RemoveStyleFunc)(lv_obj_t *obj);
    static const RemoveStyleFunc remove_style_funcs[] = {
        remove_style_number_display,
    };
    remove_style_funcs[styleIndex](obj);
}

