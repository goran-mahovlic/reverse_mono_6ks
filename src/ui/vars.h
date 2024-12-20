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
    FLOW_GLOBAL_VARIABLE_COEFFICIENT = 0
};

// Native global variables

extern bool get_var_intial_home_z();
extern void set_var_intial_home_z(bool value);
extern bool get_var_calibrated();
extern void set_var_calibrated(bool value);
extern bool get_var_running();
extern void set_var_running(bool value);
extern bool get_var_positive_diff();
extern void set_var_positive_diff(bool value);
extern double get_var_max_diff();
extern void set_var_max_diff(double value);
extern double get_var_sensor();
extern void set_var_sensor(double value);
extern double get_var_current_diff();
extern void set_var_current_diff(double value);
extern double get_var_sensor_min();
extern void set_var_sensor_min(double value);
extern double get_var_sensor_max();
extern void set_var_sensor_max(double value);
extern int32_t get_var_motor_speed();
extern void set_var_motor_speed(int32_t value);
extern int32_t get_var_current_position();
extern void set_var_current_position(int32_t value);
extern int32_t get_var_loops();
extern void set_var_loops(int32_t value);
extern const char *get_var_current_operation();
extern void set_var_current_operation(const char *value);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/