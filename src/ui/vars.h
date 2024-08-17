#ifndef EEZ_LVGL_UI_VARS_H
#define EEZ_LVGL_UI_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// enum declarations



// Flow global variables

enum FlowGlobalVariables {
    FLOW_GLOBAL_VARIABLE_NONE
};

// Native global variables

extern int32_t get_var_motor_speed();
extern void set_var_motor_speed(int32_t value);
extern const char *get_var_current_operation();
extern void set_var_current_operation(const char *value);
extern int32_t get_var_current_position();
extern void set_var_current_position(int32_t value);
extern bool get_var_intial_home_z();
extern void set_var_intial_home_z(bool value);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/