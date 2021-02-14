
#include <stdio.h>
#include <stdbool.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_task_wdt.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include "global.h"
#include "properties.h"
#include "stepper_driver.h"
#include "conversion_functions.h"

static void wait_timing_ms(int t_ms);

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE_US        (TIMER_BASE_CLK / TIMER_DIVIDER / 1000000)  // convert counter value to micro seconds
#define LOG_TAG "STEPPER_DRIVER"

enum STEPPER_MOTOR { MOTOR_X, MOTOR_Y };
typedef enum STEPPER_MOTOR stepper_motor_t;

typedef struct {
    uint64_t x_space;
    uint64_t x_mark;
    uint64_t y_space;
    uint64_t y_mark;
} interrupt_timing_t;

static interrupt_timing_t int_timing;

static int x_state = 0;
static int y_state = 0;
static int x_dir = 1;
static int y_dir = 0;
static int x_step_count = 0;
static int y_step_count = 0;
static uint64_t timer_counter_save_x;
static uint64_t timer_counter_save_y;

static bool stepper_run = false;

typedef struct {
    timer_group_t timer_grp;
    timer_idx_t timer_idx;
} hw_timer_def_t;

hw_timer_def_t hw_timer_def[] = { {TIMER_GROUP_0, TIMER_0}, {TIMER_GROUP_1, TIMER_1} };

static inline void send_step(stepper_motor_t ch)
{
    if (ch == MOTOR_X) {
        if (!stepper_run && !x_state) {
            return;
        }
        x_state = !x_state; // TODO Consider x_state = (!x_state) & stepper_run;
        if (x_state || stepper_driver_props.x_dedge) {
            if (x_dir) {
                x_step_count++;
            } else {
                x_step_count--;
            }
        }
        gpio_set_level(gpio_map.x_step_pin, x_state); // _-_-_
    } else { // MOTOR_Y:
        if (!stepper_run && !y_state) {
            return;
        }
        y_state = !y_state;
        if (y_state || stepper_driver_props.y_dedge) {
            if (y_dir) {
                y_step_count++;
            } else {
                y_step_count--;
            }
        }
        gpio_set_level(gpio_map.y_step_pin, y_state); // _-_-_
    }
}

static void send_dir(stepper_motor_t ch)
{
    ESP_LOGD(LOG_TAG, "send_dir: set %s dir (pin %d) to %d wait %5.2fms", ch == MOTOR_X ? "X" : "Y", ch == MOTOR_X ? gpio_map.x_dir_pin : gpio_map.y_dir_pin, ch == MOTOR_X ? x_dir : x_dir, stepper_driver_props.enbl_and_dir_setup_hold_time_ns / 9999);
    if (ch == MOTOR_X) {
        gpio_set_level(gpio_map.x_dir_pin, x_dir);
    } else { // MOTOR_Y:
        gpio_set_level(gpio_map.y_dir_pin, y_dir);
    }
    wait_timing_ms(stepper_driver_props.enbl_and_dir_setup_hold_time_ns / 9999); // TODO come back to fix this
}

static void send_sleep(bool flag) {
    ESP_LOGD(LOG_TAG, "send_sleep: set nSLEEP (pin TBD) to %d wait %5.2fms", !flag, stepper_driver_props.wake_from_sleep_time_ms);
    wait_timing_ms(stepper_driver_props.wake_from_sleep_time_ms);
}

static void send_enable(bool flag) {
    ESP_LOGD(LOG_TAG, "send_enable: set nENABLE (pin %d) to %d wait %5.2fms", gpio_map.step_nenable_pin, !flag, stepper_driver_props.enbl_and_dir_setup_hold_time_ns / 9999);
    gpio_set_level(gpio_map.step_nenable_pin, !flag);
    wait_timing_ms(stepper_driver_props.enbl_and_dir_setup_hold_time_ns / 9999);
}

static void send_pen_lift(bool flag) {
    ESP_LOGD(LOG_TAG, "send_pen_lift: set LIFT (pin %d) to %d", gpio_map.pen_lift_pin, !flag);
    gpio_set_level(gpio_map.pen_lift_pin, !flag);
}

static inline bool is_space() {
    return x_state == 0 && y_state == 0;
}

static void wait_timing_ms(int t_ms) {
    vTaskDelay(t_ms / portTICK_PERIOD_MS); // Does 0 yield or is there a yield
}

static inline uint64_t next_pulse(stepper_motor_t ch) {
    if (ch == MOTOR_X) {
        if (x_state) {
            return int_timing.x_mark;
        } else {
            return int_timing.x_space;
        }
    } else { // MOTOR_Y:
        if (y_state) {
            return int_timing.y_mark;
        } else {
            return int_timing.y_space;
        }
    }
}

void IRAM_ATTR timer_group_isr(void *para)
{
    hw_timer_def_t *timer_ptr =  (hw_timer_def_t *)para;
    timer_spinlock_take(timer_ptr->timer_grp);

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(timer_ptr->timer_grp);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(timer_ptr->timer_grp, timer_ptr->timer_idx);

    /* Clear the interrupt
       and update the alarm time */
    if (timer_intr & TIMER_INTR_T0) {
        timer_group_clr_intr_status_in_isr(timer_ptr->timer_grp, TIMER_0);
        send_step(MOTOR_X);
        timer_counter_value += next_pulse(MOTOR_X);
        timer_counter_save_x = timer_counter_value;
        timer_group_set_alarm_value_in_isr(timer_ptr->timer_grp, timer_ptr->timer_idx, timer_counter_value);
    } else if (timer_intr & TIMER_INTR_T1) {
        timer_group_clr_intr_status_in_isr(timer_ptr->timer_grp, TIMER_1);
        send_step(MOTOR_Y);
        timer_counter_value += next_pulse(MOTOR_Y);
        timer_counter_save_y = timer_counter_value;
        timer_group_set_alarm_value_in_isr(timer_ptr->timer_grp, timer_ptr->timer_idx, timer_counter_value);
    }

    if (stepper_run) {
        /* After the alarm has been triggered
        we need enable it again, so it is triggered the next time */
        timer_group_enable_alarm_in_isr(timer_ptr->timer_grp, timer_ptr->timer_idx);
    }

    timer_spinlock_give(timer_ptr->timer_grp);
}

static void timer_group_init(hw_timer_def_t *timer_ptr, double timer_interval_usec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 0,
    }; // default clock source is APB
    timer_init(timer_ptr->timer_grp, timer_ptr->timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(timer_ptr->timer_grp, timer_ptr->timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(timer_ptr->timer_grp, timer_ptr->timer_idx, timer_interval_usec * TIMER_SCALE_US);
    timer_enable_intr(timer_ptr->timer_grp, timer_ptr->timer_idx);
    timer_isr_register(timer_ptr->timer_grp, timer_ptr->timer_idx, timer_group_isr, (void *) timer_ptr, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(timer_ptr->timer_grp, timer_ptr->timer_idx);
}

void line_init() {
    send_dir(MOTOR_X);
    send_dir(MOTOR_Y);
    send_pen_lift(true);
}

void stepper_motors_run() {
    send_sleep(false); // Wake up
    send_enable(true);
    stepper_run = true;
    line_init();
    timer_group_init(&hw_timer_def[0], stepper_driver_props.step_high_low_duration_us);
    timer_group_init(&hw_timer_def[1], stepper_driver_props.step_high_low_duration_us);
}

void stepper_motors_stop() {
     // TODO stop when all are ... !is_space()
    stepper_run = false;
}

int dump_stepper_state(char *txtbuf, size_t bufsz) {
    return snprintf(txtbuf, bufsz, "x steps, %d, %d, %d, y steps, %d, %d, %d, position mm, %5.2f, %5.2f, timer counters, %llu, %llu",
    x_step_count, x_dir, x_state,
    y_step_count, y_dir, y_state,
    map_step_to_position(x_step_count), map_step_to_position(y_step_count),
    timer_counter_save_x,
    timer_counter_save_y);
}

int dump_stepper_timing(char *txtbuf, size_t bufsz) {
    return snprintf(txtbuf, bufsz, "base clock, %d, counts per second, %d, counts per microsecond, %d", TIMER_BASE_CLK, TIMER_BASE_CLK / TIMER_DIVIDER, TIMER_SCALE_US);
}

int dump_interrupt_timing(char *txtbuf, size_t bufsz) {
    return snprintf(txtbuf, bufsz, "interrupt x, %llu, %llu, y, %llu, %llu, period x, %5.2fus, y, %5.2fus, freq x, %5.2fHz, y, %5.2fHz", 
            int_timing.x_space, int_timing.x_mark, int_timing.y_space, int_timing.y_mark,
            (float)(int_timing.x_space + int_timing.x_mark) / (TIMER_BASE_CLK / TIMER_DIVIDER) * 1000 * 1000, (float)(int_timing.y_space + int_timing.y_mark) / (TIMER_BASE_CLK / TIMER_DIVIDER) * 1000 * 1000,
            (float)(TIMER_BASE_CLK / TIMER_DIVIDER) / (int_timing.x_space + int_timing.x_mark), (float)(TIMER_BASE_CLK / TIMER_DIVIDER) / (int_timing.y_space + int_timing.y_mark));
}

// 12500 => 200 steps per second
void set_initial_interrupt_timing(uint64_t x_space, uint64_t x_mark, uint64_t y_space, uint64_t y_mark) {
    int_timing.x_space = x_space;
    int_timing.x_mark = x_mark;
    int_timing.y_space = y_space;
    int_timing.y_mark = y_mark;
 }

