#include "Bluetooth.hpp"
#include "Config.hpp"
#include "Device.hpp"
#include "Display.hpp"
#include "I2CBus.hpp"
#include "LogModel.hpp"
#include "TestModel.hpp"
#include "Tracker.hpp"
#include "esp_timer.h"
#include "nvs_flash.h"

#if defined(DEVICE_BIKE)
#include "HallSpeed.hpp"
#endif

// ── Глобалы ──────────────────────────────────────────────────

Context global_context;

static SemaphoreHandle_t tracker_sem = nullptr;
static LogModel* global_log = nullptr;

// ── Задачи FreeRTOS ──────────────────────────────────────────

static void timerCallback(void*)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(tracker_sem, &woken);
    if (woken)
        portYIELD_FROM_ISR();
}

static void bluetoothTask(void*)
{
    Bluetooth bt;
    auto from = xTaskGetTickCount();
    constexpr auto period = pdMS_TO_TICKS(1000 / BTConfig::FREQUENCY);

    for (;;) {
        bt.loop();
        vTaskDelayUntil(&from, period);
    }
}

static void speedTask(void* param)
{
    auto* sensor = static_cast<ISpeedSource*>(param);
    while (!sensor->begin())
        vTaskDelay(pdMS_TO_TICKS(1000));

    auto from = xTaskGetTickCount();
    constexpr auto period = pdMS_TO_TICKS(CANConfig::POLL_INTERVAL_MS);

    for (;;) {
        SpeedData s = sensor->readSpeed();
        if (s.valid) {
            uint8_t idx = global_context.active_speed_idx.load(std::memory_order_acquire) ^ 1;
            global_context.speed_buffer[idx] = s;
            global_context.active_speed_idx.store(idx, std::memory_order_release);
        }
        vTaskDelayUntil(&from, period);
    }
}

static void oledTask(void* param)
{
    auto* tm = static_cast<TestModel*>(param);
    Display display;
    while (!display.begin())
        vTaskDelay(pdMS_TO_TICKS(1000));

    char l1[24], l2[24], l3[24], l4[24], l5[24], l6[24];
    auto from = xTaskGetTickCount();

    for (;;) {
        bool ble = global_context.ble_connected.load(std::memory_order_relaxed);
        snprintf(l1, sizeof(l1), "GPFix TEST  BT:%s", ble ? "OK" : "--");

        bool sd = global_log && global_log->isReady();
        snprintf(l2, sizeof(l2), "SD:%s R:%lu D:%lu",
            sd ? "REC" : "ERR",
            static_cast<unsigned long>(global_log ? global_log->records_written : 0),
            static_cast<unsigned long>(global_log ? global_log->records_dropped.load() : 0));

#if defined(DEVICE_BIKE)
        uint32_t pulses = HallSpeed::hallPulseCount;
#else
        uint32_t pulses = tm->pulse_count;
#endif
        snprintf(l3, sizeof(l3), "Spd:%.1fkm/h P:%lu",
            tm->speed_kmh, static_cast<unsigned long>(pulses));

        snprintf(l4, sizeof(l4), "G:%.2f %.2f %.2f",
            tm->u_z(0), tm->u_z(1), tm->u_z(2));

        snprintf(l5, sizeof(l5), "Y:%.0f\xb0 W:%.1f\xb0/s %s",
            tm->current_yaw_deg, tm->w_proj,
            tm->is_leveled ? "OK" : "CAL");

        if (tm->gps_valid)
            snprintf(l6, sizeof(l6), "GPS:%.4f,%.4f", tm->gps_lat, tm->gps_lon);
        else
            snprintf(l6, sizeof(l6), "GPS: Wait phone");

        display.render(l1, l2, l3, l4, l5, l6);
        vTaskDelayUntil(&from, pdMS_TO_TICKS(250));
    }
}

static void trackerTask(void* param)
{
    auto* model = static_cast<IModel*>(param);
    constexpr auto freq = IMUConfig::SAMPLE_RATE_HZ;
    constexpr uint64_t loop_us = 1000000 / freq;

    Tracker tracker(model, freq);
    tracker_sem = xSemaphoreCreateBinary();

    const esp_timer_create_args_t args = {
        .callback = &timerCallback,
        .name = "tracker_timer"
    };
    esp_timer_handle_t th;
    esp_timer_create(&args, &th);
    esp_timer_start_periodic(th, loop_us);

    for (;;) {
        if (xSemaphoreTake(tracker_sem, portMAX_DELAY) == pdTRUE) {
            tracker.loop();
        }
    }
}

// ── Entry point ──────────────────────────────────────────────

void setup()
{
    I2CBus::init();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    static CurrentDevice device;
    ESP_LOGI("MAIN", "Device: %s", device.name());

    // ── Модели ───────────────────────────────────────────────
    auto* test_model = new TestModel();

    auto* log_model = new LogModel();
    global_log = log_model;
    if (!log_model->begin())
        ESP_LOGW("MAIN", "SD logger init failed");

    static auto* model = new CompositeModel(test_model, log_model);

    // ── Задачи ───────────────────────────────────────────────
    xTaskCreatePinnedToCore(bluetoothTask, "bt", TaskConfig::BLE_STACK, nullptr, TaskConfig::BLE_PRIO, nullptr, TaskConfig::BLE_CORE);
    xTaskCreatePinnedToCore(speedTask, "speed", TaskConfig::CAN_STACK, device.getSpeedSource(), TaskConfig::CAN_PRIO, nullptr, TaskConfig::CAN_CORE);
    xTaskCreatePinnedToCore(oledTask, "oled", 4096, test_model, 1, nullptr, 0);
    xTaskCreatePinnedToCore(trackerTask, "tracker", TaskConfig::IMU_STACK, model, TaskConfig::IMU_PRIO, nullptr, TaskConfig::IMU_CORE);
}

void loop()
{
    vTaskDelete(nullptr);
}
