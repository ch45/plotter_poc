
typedef struct {
    int x_step_pin;
    int x_dir_pin;
    int x_limit_pin;
    int y_step_pin;
    int y_dir_pin;
    int y_limit_pin;
    int pen_lift_pin;
    int step_nenable_pin;    
} gpio_mapping_t;

typedef struct {
    fpnumber_t wake_from_sleep_time_ms;    
    fpnumber_t enbl_and_dir_setup_hold_time_ns;
    fpnumber_t step_high_low_duration_us;
    int x_dedge;
    int y_dedge;
    int micro_steps; //  N where 1/N is the microstep resolution
} stepper_driver_prop_t;

typedef struct {
    int steps_per_rotation;
} stepper_motor_prop_t;

typedef struct {
    fpnumber_t wheel_dia_mm;
    fpnumber_t pen_thickness_mm;
} plotter_prop_t;

int dump_some_properties(char *txtbuf, size_t bufsz);

extern gpio_mapping_t gpio_map;
extern stepper_driver_prop_t stepper_driver_props;
extern stepper_motor_prop_t stepper_motor_props;
extern plotter_prop_t plotter_props;
