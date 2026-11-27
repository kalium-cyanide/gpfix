#include "BicycleESKF.hpp"
#include "Bluetooth.hpp"
#include "Config.hpp"
#include "Device.hpp"
#include "Display.hpp"
#include "I2CBus.hpp"
#include "LogModel.hpp"
#include "Tracker.hpp"
#include "esp_timer.h"
#include "nvs_flash.h"

#if defined(DEVICE_BIKE)
#include "CycplusSpeed.hpp"
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
    auto* eskf = static_cast<BicycleESKF*>(param);
    static Display display;
    while (!display.begin())
        vTaskDelay(pdMS_TO_TICKS(1000));

    char l1[24], l2[24], l3[24], l4[24], l5[24], l6[24];
    auto from = xTaskGetTickCount();

    for (;;) {
        bool ble = global_context.ble_connected.load(std::memory_order_relaxed);
        snprintf(l1, sizeof(l1), "GPFix ESKF  BT:%s", ble ? "OK" : "--");

        if (!SDConfig::ENABLED) {
            snprintf(l2, sizeof(l2), "SD:OFF (No HW)");
        } else {
            bool sd = global_log && global_log->isReady();
            snprintf(l2, sizeof(l2), "SD:%s R:%lu D:%lu",
                sd ? "REC" : "ERR",
                static_cast<unsigned long>(global_log ? global_log->records_written : 0),
                static_cast<unsigned long>(global_log ? global_log->records_dropped.load() : 0));
        }

#if defined(DEVICE_BIKE)
        uint32_t pulses = BikeConfig::USE_BLE_SPEED
            ? CycplusSpeed::wheelRevs.load(std::memory_order_relaxed)
            : HallSpeed::hallPulseCount;
#else
        uint32_t pulses = 0;
#endif
        snprintf(l3, sizeof(l3), "Spd:%.1f P:%lu",
            eskf->X(3) * 3.6f, static_cast<unsigned long>(pulses));

        if (eskf->is_leveled) {
            snprintf(l4, sizeof(l4), "G:%.1f %.1f %.1f",
                eskf->u_z(0), eskf->u_z(1), eskf->u_z(2));
            snprintf(l5, sizeof(l5), "Y:%.0f\xb0 %s",
                eskf->X(2) * RAD2DEG_F,
                !eskf->is_fwd_calibrated ? "FWD?" : (eskf->isDR() ? "DR" : "OK"));
        } else {
            snprintf(l4, sizeof(l4), "G:WAIT IMU (LVL)");
            snprintf(l5, sizeof(l5), "Y:%.0f\xb0 CAL", eskf->X(2) * RAD2DEG_F);
        }

        GPSData gps = global_context.gps_buffer[global_context.active_gps_idx.load(std::memory_order_acquire)];
        if (gps.valid)
            snprintf(l6, sizeof(l6), "GPS:%.4f,%.4f", gps.lat, gps.lon);
        else
            snprintf(l6, sizeof(l6), "GPS: Wait phone");

        // Стрелка показывает откалиброванную продольную ось u_x ("куда смотрит нос велика" в СК платы)
        float arrow_ux = eskf->u_x(0);
        float arrow_uy = -eskf->u_x(1);
        if (std::hypot(arrow_ux, arrow_uy) < 0.2f) {
            arrow_uy = -eskf->u_x(2);
        }
        const char* ph = eskf->is_leveled ? "GO" : "?";

        display.render(l1, l2, l3, l4, l5, l6, eskf->is_fwd_calibrated, arrow_ux, arrow_uy, ph);
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
    I2CBus::scan();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    static CurrentDevice device;
    ESP_LOGI("MAIN", "Device: %s", device.name());

    if (!BLEDevice::getInitialized()) {
        BLEDevice::init(BTConfig::DEVICE_NAME);
        BLEDevice::setMTU(BTConfig::MTU_SIZE);
    }

    // ── Модели ───────────────────────────────────────────────
    auto* eskf_model = new BicycleESKF(device.getModelParams());
    auto* log_model  = new LogModel();
    global_log = log_model;

    if constexpr (SDConfig::ENABLED) {
        if (!log_model->begin()) {
            ESP_LOGW("MAIN", "SD logger init failed — proceeding without SD recording");
        }
    } else {
        ESP_LOGI("MAIN", "SD logger disabled by configuration");
    }

    static auto* model = new CompositeModel(eskf_model, log_model);

    // ── Задачи ───────────────────────────────────────────────
    xTaskCreatePinnedToCore(bluetoothTask, "bt", TaskConfig::BLE_STACK, nullptr, TaskConfig::BLE_PRIO, nullptr, TaskConfig::BLE_CORE);
    xTaskCreatePinnedToCore(speedTask, "speed", TaskConfig::CAN_STACK, device.getSpeedSource(), TaskConfig::CAN_PRIO, nullptr, TaskConfig::CAN_CORE);
    xTaskCreatePinnedToCore(oledTask, "oled", 4096, eskf_model, 1, nullptr, 0);
    xTaskCreatePinnedToCore(trackerTask, "tracker", TaskConfig::IMU_STACK, model, TaskConfig::IMU_PRIO, nullptr, TaskConfig::IMU_CORE);
}

void loop()
{
    vTaskDelete(nullptr);
}
