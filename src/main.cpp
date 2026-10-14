#include "bluetooth.hpp"
#include "config.hpp"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "tracker.hpp"

SemaphoreHandle_t tracker_sem = NULL;

Context global_context;

void timer_callback(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(tracker_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void bluetoothSetup(void*)
{
    {
        auto instance = Bluetooth();

        constexpr auto frequency = BTConfig::FREQUENCY;

        auto from = xTaskGetTickCount();
        constexpr auto duration = pdMS_TO_TICKS((1 * 1000) / frequency);

        while (true) {
            instance.loop();

            vTaskDelayUntil(&from, duration);
        }
    }
    vTaskDelete(NULL);
}

void trackerSetup(void*)
{
    {
        constexpr auto frequency = IMUConfig::SAMPLE_RATE_HZ;
        constexpr uint64_t loop_time = 1000000 / frequency;

        auto instance = Tracker(frequency);

        tracker_sem = xSemaphoreCreateBinary();

        const esp_timer_create_args_t timer_args = {
            .callback = &timer_callback,
            .name = "tracker_timer"
        };
        esp_timer_handle_t timer_handle;
        esp_timer_create(&timer_args, &timer_handle);
        esp_timer_start_periodic(timer_handle, loop_time);

        while (true) {
            if (xSemaphoreTake(tracker_sem, portMAX_DELAY) == pdTRUE) {
                instance.loop();
            }
        }
    }
    vTaskDelete(NULL);
}

void setup()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    xTaskCreatePinnedToCore(bluetoothSetup, "bluetooth", TaskConfig::BLE_STACK, nullptr, TaskConfig::BLE_PRIO, nullptr, TaskConfig::BLE_CORE);
    xTaskCreatePinnedToCore(trackerSetup, "tracker", TaskConfig::IMU_STACK, nullptr, TaskConfig::IMU_PRIO, nullptr, TaskConfig::IMU_CORE);
}

void loop()
{
    vTaskDelete(NULL);
}