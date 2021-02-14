/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include "global.h"
#include "properties.h"
#include "system_chip.h"
#include "stepper_driver.h"

#define RUN_SECONDS 30
#define MSG_LENGTH 200
#define LOG_TAG "MAIN"

static void poc_loop() {

    char *msg_buf = (char *) malloc(MSG_LENGTH);

    system_on_a_chip_init();

    // 12500 =>  200 steps per second
    //  2500 => 1000Hz approx test limit for motors to turn
    set_initial_interrupt_timing(10000, 10000, 12500, 12500);

    if (LOG_LOCAL_LEVEL >= ESP_LOG_INFO) dump_interrupt_timing(msg_buf, MSG_LENGTH);
    ESP_LOGI(LOG_TAG, "%s", msg_buf);

    if (LOG_LOCAL_LEVEL >= ESP_LOG_INFO) dump_some_properties(msg_buf, MSG_LENGTH);
    ESP_LOGI(LOG_TAG, "%s", msg_buf);

    if (LOG_LOCAL_LEVEL >= ESP_LOG_INFO) dump_stepper_timing(msg_buf, MSG_LENGTH);
    ESP_LOGI(LOG_TAG, "%s", msg_buf);

    if (LOG_LOCAL_LEVEL >= ESP_LOG_INFO) dump_stepper_state(msg_buf, MSG_LENGTH);
    ESP_LOGI(LOG_TAG, "%s", msg_buf);
    
    ESP_LOGI(LOG_TAG, "call stepper_motors_run");
    stepper_motors_run();
    
    for (int i = RUN_SECONDS; i >= 0; i--) {
        if ((i > 5 && i % 5 == 0) || i <= 5) {
            printf("Restarting in %d seconds...\n", i);   
        }

        if (LOG_LOCAL_LEVEL >= ESP_LOG_INFO) dump_stepper_state(msg_buf, MSG_LENGTH);
        ESP_LOGI(LOG_TAG, "%s", msg_buf);

        if (i == 3) {
            ESP_LOGI(LOG_TAG, "call stepper_motors_stop");
            stepper_motors_stop();
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    free(msg_buf);
}

void app_main(void)
{

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    poc_loop();
         
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
