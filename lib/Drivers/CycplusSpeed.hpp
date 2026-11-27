#pragma once

#include "Config.hpp"
#include "Data.hpp"
#include "ISpeedSource.hpp"
#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEUtils.h>
#include <atomic>
#include <cstring>
#include <esp_log.h>
#include <esp_timer.h>

// ── BLE-датчик скорости CYCPLUS C3 (профиль CSC 0x1816 / 0x2A5B) ──
//
//  Подключается как BLE-клиент к устройству с именем BikeConfig::CSC_DEVICE_NAME
//  ("CYCPLUS C3 15481") или сервисом 0x1816 (Cycling Speed and Cadence).
//  Подписывается на уведомления характеристики 0x2A5B (CSC Measurement)
//  и вычисляет скорость по дельте оборотов колеса и времени (1/1024 с).

struct CycplusSpeed : public ISpeedSource {

    static inline constexpr uint16_t CSC_SERVICE_UUID = 0x1816;
    static inline constexpr uint16_t CSC_MEAS_UUID    = 0x2A5B;

    static inline std::atomic<uint32_t> wheelRevs { 0 };

    static inline CycplusSpeed* instance_ = nullptr;

    BLEClient*                 client_       = nullptr;
    BLERemoteCharacteristic*   meas_char_    = nullptr;
    BLEAdvertisedDevice*       target_dev_   = nullptr;

    std::atomic<bool>          connected_    { false };
    std::atomic<bool>          found_device_ { false };

    portMUX_TYPE               mux_          = portMUX_INITIALIZER_UNLOCKED;
    float                      speed_kmh_    = 0.0f;
    uint64_t                   last_pkt_us_  = 0;

    bool                       has_prev_     = false;
    uint32_t                   prev_revs_    = 0;
    uint16_t                   prev_time_    = 0;

    class ClientCallbacks : public BLEClientCallbacks {
        CycplusSpeed& owner_;
    public:
        explicit ClientCallbacks(CycplusSpeed& o) : owner_(o) {}
        void onConnect(BLEClient*) override
        {
            owner_.connected_.store(true, std::memory_order_release);
            ESP_LOGI("CYCPLUS", "Connected to CYCPLUS C3");
        }
        void onDisconnect(BLEClient*) override
        {
            owner_.connected_.store(false, std::memory_order_release);
            owner_.has_prev_ = false;
            portENTER_CRITICAL(&owner_.mux_);
            owner_.speed_kmh_ = 0.0f;
            portEXIT_CRITICAL(&owner_.mux_);
            ESP_LOGW("CYCPLUS", "Disconnected from CYCPLUS C3 — will re-scan");
        }
    };

    class ScanCallbacks : public BLEAdvertisedDeviceCallbacks {
        CycplusSpeed& owner_;
    public:
        explicit ScanCallbacks(CycplusSpeed& o) : owner_(o) {}
        void onResult(BLEAdvertisedDevice dev) override
        {
            bool name_match = dev.haveName()
                && (dev.getName() == BikeConfig::CSC_DEVICE_NAME
                    || std::strncmp(dev.getName().c_str(), "CYCPLUS", 7) == 0);
            bool svc_match = dev.haveServiceUUID()
                && dev.isAdvertisingService(BLEUUID(CSC_SERVICE_UUID));

            if (name_match || (svc_match && std::strlen(BikeConfig::CSC_DEVICE_NAME) == 0)) {
                ESP_LOGI("CYCPLUS", "Found sensor: %s [%s]",
                         dev.haveName() ? dev.getName().c_str() : "?",
                         dev.getAddress().toString().c_str());
                delete owner_.target_dev_;
                owner_.target_dev_ = new BLEAdvertisedDevice(dev);
                owner_.found_device_.store(true, std::memory_order_release);
                BLEDevice::getScan()->stop();
            }
        }
    };

    ClientCallbacks client_cb_;
    ScanCallbacks   scan_cb_;

    CycplusSpeed() : client_cb_(*this), scan_cb_(*this) { instance_ = this; }

    ~CycplusSpeed() override
    {
        delete target_dev_;
        target_dev_ = nullptr;
    }

    static void notifyCallback(BLERemoteCharacteristic*, uint8_t* data, size_t len, bool)
    {
        if (instance_) instance_->onMeasurement(data, len);
    }

    void onMeasurement(const uint8_t* data, size_t len)
    {
        if (!data || len < 1) return;

        uint8_t flags = data[0];
        if ((flags & 0x01) == 0 || len < 7) return; // Нет данных колеса

        uint32_t revs = static_cast<uint32_t>(data[1])
                      | (static_cast<uint32_t>(data[2]) << 8)
                      | (static_cast<uint32_t>(data[3]) << 16)
                      | (static_cast<uint32_t>(data[4]) << 24);
        uint16_t evt_time = static_cast<uint16_t>(data[5])
                          | (static_cast<uint16_t>(data[6]) << 8);

        uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
        wheelRevs.store(revs, std::memory_order_relaxed);

        float new_speed = 0.0f;
        if (has_prev_) {
            uint32_t d_rev  = revs - prev_revs_;
            uint16_t d_time = static_cast<uint16_t>(evt_time - prev_time_);

            if (d_rev > 0 && d_rev < 100 && d_time > 0) {
                float dt_s = static_cast<float>(d_time) / 1024.0f;
                new_speed = (static_cast<float>(d_rev) * BikeConfig::WHEEL_CIRCUMFERENCE_M / dt_s) * 3.6f;
            }
        }

        prev_revs_ = revs;
        prev_time_ = evt_time;
        has_prev_  = true;

        portENTER_CRITICAL(&mux_);
        speed_kmh_   = new_speed;
        last_pkt_us_ = now_us;
        portEXIT_CRITICAL(&mux_);
    }

    bool connectToTarget()
    {
        if (!target_dev_) return false;

        if (!client_) {
            client_ = BLEDevice::createClient();
            client_->setClientCallbacks(&client_cb_);
        }

        if (!client_->connect(target_dev_)) {
            ESP_LOGW("CYCPLUS", "BLE connect failed");
            return false;
        }

        BLERemoteService* svc = client_->getService(BLEUUID(CSC_SERVICE_UUID));
        if (!svc) {
            ESP_LOGW("CYCPLUS", "CSC service 0x1816 not found");
            client_->disconnect();
            return false;
        }

        meas_char_ = svc->getCharacteristic(BLEUUID(CSC_MEAS_UUID));
        if (!meas_char_ || !meas_char_->canNotify()) {
            ESP_LOGW("CYCPLUS", "CSC char 0x2A5B not found or not notifiable");
            client_->disconnect();
            return false;
        }

        meas_char_->registerForNotify(notifyCallback);
        ESP_LOGI("CYCPLUS", "Subscribed to CSC 0x2A5B");
        return true;
    }

    void tryScanAndConnect()
    {
        if (connected_.load(std::memory_order_acquire)) return;

        found_device_.store(false, std::memory_order_release);
        BLEScan* scan = BLEDevice::getScan();
        scan->setAdvertisedDeviceCallbacks(&scan_cb_);
        scan->setActiveScan(true);
        scan->setInterval(100);
        scan->setWindow(99);
        scan->start(2, false);

        if (found_device_.load(std::memory_order_acquire)) {
            connectToTarget();
        }
        if (!global_context.ble_connected.load(std::memory_order_relaxed)) {
            BLEDevice::startAdvertising();
        }
    }

    bool begin() override
    {
        if (!BLEDevice::getInitialized()) {
            BLEDevice::init(BTConfig::DEVICE_NAME);
            BLEDevice::setMTU(BTConfig::MTU_SIZE);
        }
        ESP_LOGI("CYCPLUS", "Looking for BLE sensor \"%s\" (wheel %.3f m)",
                 BikeConfig::CSC_DEVICE_NAME, BikeConfig::WHEEL_CIRCUMFERENCE_M);
        return true; // Первый скан начнётся в readSpeed() после старта BLE-сервера
    }

    SpeedData readSpeed() override
    {
        static uint64_t last_scan_us = 0;
        uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());

        if (!connected_.load(std::memory_order_acquire)) {
            if (now_us - last_scan_us >= 5000000ULL) { // Скан раз в 5 с при отсутствии связи
                last_scan_us = now_us;
                tryScanAndConnect();
            }
            return { 0.0f, now_us, false };
        }

        float spd = 0.0f;
        uint64_t pkt_us = 0;
        portENTER_CRITICAL(&mux_);
        spd    = speed_kmh_;
        pkt_us = last_pkt_us_;
        portEXIT_CRITICAL(&mux_);

        if (pkt_us == 0) {
            return { 0.0f, now_us, false };
        }

        if (now_us - pkt_us > BikeConfig::HALL_TIMEOUT_US) {
            spd = 0.0f;
        }

        return { spd, now_us, true };
    }
};
