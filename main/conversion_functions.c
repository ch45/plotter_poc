
#include <math.h>
#include "global.h"
#include "properties.h"
#include "conversion_functions.h"

fpnumber_t map_step_to_position(int step_count) {
    return plotter_props.wheel_dia_mm * M_PI * step_count / (stepper_driver_props.micro_steps * stepper_motor_props.steps_per_rotation);
}
