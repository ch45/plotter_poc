
#include "driver/gpio.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include "global.h"
#include "properties.h"
#include "system_chip.h"

#define LOG_TAG "SYSTEM_CHIP"

static void system_gpio_init() {
    gpio_config_t io_conf;

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL << gpio_map.x_step_pin) | (1ULL << gpio_map.y_step_pin) | \
                            (1ULL << gpio_map.x_dir_pin) | (1ULL << gpio_map.y_dir_pin) | \
                            (1ULL << gpio_map.pen_lift_pin) | (1ULL << gpio_map.step_nenable_pin));
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;


    ESP_LOGD(LOG_TAG, "system_gpio_init io_conf.pin_bit_mask, %llx", io_conf.pin_bit_mask);

    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void system_on_a_chip_init() {
    system_gpio_init();
}
