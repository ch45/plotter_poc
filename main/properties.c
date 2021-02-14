
#include <math.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "global.h"
#include "properties.h"

gpio_mapping_t gpio_map = {
    GPIO_NUM_25, // X_STEP_PIN
    GPIO_NUM_26, // X_DIR_PIN
    GPIO_NUM_34, // X_LIMIT_PIN
    GPIO_NUM_32, // Y_STEP_PIN
    GPIO_NUM_33, // Y_DIR_PIN
    GPIO_NUM_35, // Y_LIMIT_PIN
    GPIO_NUM_27, // PEN_nLIFT_PIN
    GPIO_NUM_2   // STEP_nENABLE_PIN
};

stepper_driver_prop_t stepper_driver_props = {
    1.7, // WAKE_FROM_SLEEP_TIME_ms          ms MAX
    650, // ENBL_AND_DIR_SETUP_HOLD_TIME_ns  ns MIN
    1.9, // STEP_HIGH_LOW_DURATION_us        us MIN
    0,   // X Double Edge
    0,   // Y Double Edge
    1    // MICRO_STEPS
};

stepper_motor_prop_t stepper_motor_props = {
    200
};

plotter_prop_t plotter_props = {
    20, // Wheel diameter mm
    0.2 // Pen thickness mm
};

int dump_some_properties(char *txtbuf, size_t bufsz) {
    return snprintf(txtbuf, bufsz, "wheel circumference, %5.2f, steps per rotation, %d, micro-stepping, 1/%d", plotter_props.wheel_dia_mm * M_PI, stepper_motor_props.steps_per_rotation , stepper_driver_props.micro_steps);
}
