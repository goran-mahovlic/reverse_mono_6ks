#if defined(EEZ_FOR_LVGL)
#include <eez/core/vars.h>
#endif

#include "ui.h"
#include "screens.h"
#include "images.h"
#include "actions.h"
#include "vars.h"

// ASSETS DEFINITION
const uint8_t assets[400] = {
    0x7E, 0x65, 0x65, 0x7A, 0x03, 0x00, 0x06, 0x00, 0xE8, 0x04, 0x00, 0x00, 0x6E, 0x24, 0x00, 0x00,
    0x00, 0x24, 0x00, 0x01, 0x00, 0x17, 0x20, 0x0C, 0x00, 0x53, 0xE0, 0x01, 0x40, 0x01, 0x01, 0x28,
    0x00, 0x13, 0x02, 0x1C, 0x00, 0x00, 0x08, 0x00, 0x13, 0x1C, 0x08, 0x00, 0x26, 0x70, 0x04, 0x2C,
    0x00, 0x11, 0x10, 0x06, 0x00, 0xB3, 0xFF, 0xFF, 0x14, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00,
    0x5C, 0x24, 0x00, 0x00, 0x08, 0x00, 0x57, 0x06, 0x00, 0x00, 0x00, 0x58, 0x58, 0x00, 0x00, 0x54,
    0x00, 0x2E, 0x3C, 0x04, 0x7C, 0x00, 0x5B, 0x09, 0x00, 0x00, 0x00, 0x48, 0x28, 0x00, 0x2E, 0x18,
    0x04, 0x28, 0x00, 0xD3, 0x64, 0x65, 0x66, 0x61, 0x75, 0x6C, 0x74, 0x00, 0xFF, 0xFF, 0x08, 0x42,
    0x3C, 0x5C, 0x00, 0xF6, 0x27, 0x74, 0x00, 0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0xAC, 0x00, 0x00,
    0x00, 0xC8, 0x00, 0x00, 0x00, 0xEC, 0x00, 0x00, 0x00, 0x08, 0x01, 0x00, 0x00, 0x24, 0x01, 0x00,
    0x00, 0x40, 0x01, 0x00, 0x00, 0x5C, 0x01, 0x00, 0x00, 0x78, 0x01, 0x00, 0x00, 0x94, 0x01, 0x00,
    0x00, 0xB0, 0x01, 0x00, 0x00, 0xCC, 0x01, 0x00, 0x00, 0x20, 0x4E, 0x54, 0x00, 0x11, 0x03, 0xF0,
    0x00, 0x06, 0x10, 0x00, 0x5B, 0xFF, 0xFF, 0x00, 0x00, 0x21, 0x20, 0x00, 0x1B, 0xCC, 0x20, 0x00,
    0x1B, 0x22, 0x20, 0x00, 0x1B, 0xB8, 0x20, 0x00, 0x1B, 0x23, 0x20, 0x00, 0x22, 0xA4, 0x01, 0xD4,
    0x00, 0x2F, 0xA8, 0x01, 0x40, 0x00, 0x03, 0x1B, 0x94, 0x40, 0x00, 0x22, 0x06, 0x04, 0x30, 0x00,
    0x2A, 0x88, 0x01, 0x14, 0x01, 0x13, 0x7C, 0x40, 0x00, 0x10, 0x01, 0xF8, 0x00, 0x1E, 0x01, 0xC8,
    0x00, 0x1F, 0x64, 0xC8, 0x00, 0x0C, 0x1B, 0x50, 0x20, 0x00, 0x1B, 0x24, 0x20, 0x00, 0x1F, 0x3C,
    0xC8, 0x00, 0x0C, 0x13, 0x28, 0xC8, 0x00, 0x1F, 0x2C, 0xC8, 0x00, 0x04, 0x1F, 0x18, 0x28, 0x01,
    0x0C, 0x1F, 0x04, 0x20, 0x00, 0x0C, 0x2F, 0xF0, 0x00, 0x20, 0x00, 0x0B, 0x2F, 0xDC, 0x00, 0x28,
    0x01, 0x03, 0x1B, 0xD0, 0x3C, 0x02, 0x26, 0xC4, 0x00, 0x28, 0x01, 0x1F, 0xBC, 0x04, 0x00, 0x24,
    0x17, 0xC4, 0x04, 0x00, 0x00, 0x01, 0x00, 0x13, 0xC0, 0x60, 0x02, 0x1F, 0xD8, 0x04, 0x00, 0x20,
    0x1F, 0xE0, 0x04, 0x00, 0x1C, 0x13, 0x00, 0xEC, 0x00, 0x2F, 0xE4, 0x00, 0x3D, 0x00, 0x1F, 0x13,
    0x01, 0xAC, 0x00, 0x1C, 0x00, 0x3C, 0x00, 0x07, 0x20, 0x01, 0x13, 0x00, 0xA4, 0x03, 0x13, 0x09,
    0xD4, 0x00, 0x1F, 0x00, 0x68, 0x00, 0x24, 0x1F, 0x5C, 0x3C, 0x00, 0x24, 0x0C, 0x8C, 0x00, 0x1B,
    0x01, 0x8C, 0x00, 0x13, 0x08, 0x04, 0x00, 0x13, 0x05, 0x08, 0x00, 0x13, 0x14, 0x84, 0x04, 0x13,
    0x03, 0x04, 0x00, 0x0F, 0x01, 0x00, 0x01, 0x07, 0x5B, 0x02, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00
};

native_var_t native_vars[] = {
    { NATIVE_VAR_TYPE_NONE, 0, 0 },
};


ActionExecFunc actions[] = {
    0
};


#if defined(EEZ_FOR_LVGL)

void ui_init() {
    eez_flow_init(assets, sizeof(assets), (lv_obj_t **)&objects, sizeof(objects), images, sizeof(images), actions);
}

void ui_tick() {
    eez_flow_tick();
    tick_screen(g_currentScreen);
}

#else

static int16_t currentScreen = -1;

static lv_obj_t *getLvglObjectFromIndex(int32_t index) {
    if (index == -1) {
        return 0;
    }
    return ((lv_obj_t **)&objects)[index];
}

static const void *getLvglImageByName(const char *name) {
    for (size_t imageIndex = 0; imageIndex < sizeof(images) / sizeof(ext_img_desc_t); imageIndex++) {
        if (strcmp(images[imageIndex].name, name) == 0) {
            return images[imageIndex].img_dsc;
        }
    }
    return 0;
}

void loadScreen(enum ScreensEnum screenId) {
    currentScreen = screenId - 1;
    lv_obj_t *screen = getLvglObjectFromIndex(currentScreen);
    lv_scr_load_anim(screen, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, false);
}

void ui_init() {
    create_screens();
    loadScreen(SCREEN_ID_MAIN);
}

void ui_tick() {
    tick_screen(currentScreen);
}

#endif
