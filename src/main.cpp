#include "bicycle_eskf.hpp"
#include "bluetooth.hpp"
#include "config.hpp"
#include "device.hpp"
#include "esp_timer.h"
#include "log_model.hpp"
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

void speedSetup(void* param)
{
    {
        ISpeedSource* sensor = static_cast<ISpeedSource*>(param);
        while (!sensor->begin()) {
            ESP_LOGW("SPEED", "Speed sensor init failed, retrying in 1s...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        auto from = xTaskGetTickCount();
        constexpr auto interval = pdMS_TO_TICKS(CANConfig::POLL_INTERVAL_MS);

        while (true) {
            SpeedData speed = sensor->readSpeed();
            if (speed.valid) {
                uint8_t next_idx = global_context.active_speed_idx.load(std::memory_order_acquire) ^ 1;
                global_context.speed_buffer[next_idx] = speed;
                global_context.active_speed_idx.store(next_idx, std::memory_order_release);
            }

            vTaskDelayUntil(&from, interval);
        }
    }
    vTaskDelete(NULL);
}

void trackerSetup(void* param)
{
    {
        IModel* model = static_cast<IModel*>(param);
        constexpr auto frequency = IMUConfig::SAMPLE_RATE_HZ;
        constexpr uint64_t loop_time = 1000000 / frequency;

        auto instance = Tracker(model, frequency);

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

    // ── Профиль устройства (Car или Bike) ────────────────────
    static CurrentDevice device;
    ESP_LOGI("MAIN", "Target device: %s", device.name());

    // ── Модели: одновременная работа ESKF и SD-логгера ──────
    auto* log_model = new LogModel();
    if (!log_model->begin()) {
        ESP_LOGW("MAIN", "SD logger init failed — proceeding without SD recording");
    }

    auto* eskf_model = new BicycleESKF(device.getModelParams());

    // CompositeModel дублирует все события (IMU, Скорость, GPS) в обе модели:
    // eskf_model  — вычисляет перемещение (DeltaS) и фильтр Калмана
    // log_model   — асинхронно пишет сырую телеметрию в фоновом таске на Core 0
    static IModel* model = new CompositeModel(eskf_model, log_model);

    xTaskCreatePinnedToCore(bluetoothSetup, "bluetooth", TaskConfig::BLE_STACK, nullptr, TaskConfig::BLE_PRIO, nullptr, TaskConfig::BLE_CORE);
    xTaskCreatePinnedToCore(speedSetup, "speed", TaskConfig::CAN_STACK, device.getSpeedSource(), TaskConfig::CAN_PRIO, nullptr, TaskConfig::CAN_CORE);
    xTaskCreatePinnedToCore(trackerSetup, "tracker", TaskConfig::IMU_STACK, model, TaskConfig::IMU_PRIO, nullptr, TaskConfig::IMU_CORE);
}

void loop()
{
    vTaskDelete(NULL);
}