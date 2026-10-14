#pragma once

#include "config.hpp"
#include "data.hpp"
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <data.pb.h>
#include <pb_decode.h>
#include <pb_encode.h>

class ServerCallbacks : public BLEServerCallbacks {
    bool& connected_ref;

public:
    explicit ServerCallbacks(bool& connected)
        : connected_ref(connected)
    {
    }

    void onConnect(BLEServer*) override
    {
        connected_ref = true;
    }

    void onDisconnect(BLEServer*) override
    {
        connected_ref = false;
        BLEDevice::startAdvertising();
    }
};

class AckCallback : public BLECharacteristicCallbacks {
public:
    AckCallback() = default;

    void onWrite(BLECharacteristic* ch, esp_ble_gatts_cb_param_t*) override
    {
        uint8_t* raw = ch->getData();
        size_t len = ch->getLength();
        if (!raw || len == 0) {
            return;
        }

        data_ack msg = data_ack_init_zero;
        pb_istream_t stream = pb_istream_from_buffer(raw, len);
        if (pb_decode(&stream, data_ack_fields, &msg)) {
            GPSData gps {};
            gps.lat = msg.lat;
            gps.lon = msg.lon;
            gps.timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
            gps.valid = true;

            uint8_t next_idx = global_context.active_gps_idx.load(std::memory_order_acquire) ^ 1;
            global_context.gps_buffer[next_idx] = gps;
            global_context.active_gps_idx.store(next_idx, std::memory_order_release);
        }
    }
};

struct Bluetooth {
    BLEServer* server = nullptr;
    BLEService* service = nullptr;
    BLECharacteristic* delta_char = nullptr;
    BLECharacteristic* ack_char = nullptr;
    BLEAdvertising* advertising = nullptr;

    bool connected = false;
    ServerCallbacks server_cb;
    AckCallback ack_cb;
    uint8_t last_delta_idx = 255;

    Bluetooth()
        : server_cb(connected)
    {
        BLEDevice::init(BTConfig::DEVICE_NAME);
        BLEDevice::setMTU(BTConfig::MTU_SIZE);

        server = BLEDevice::createServer();
        server->setCallbacks(&server_cb);

        service = server->createService(BTConfig::SERVICE_UUID);

        delta_char = service->createCharacteristic(
            BTConfig::DELTA_CHAR_UUID,
            BLECharacteristic::PROPERTY_NOTIFY
        );
        delta_char->addDescriptor(new BLE2902());

        ack_char = service->createCharacteristic(
            BTConfig::ACK_CHAR_UUID,
            BLECharacteristic::PROPERTY_WRITE
        );
        ack_char->setCallbacks(&ack_cb);

        service->start();

        advertising = BLEDevice::getAdvertising();
        advertising->addServiceUUID(BTConfig::SERVICE_UUID);
        advertising->setScanResponse(true);
        advertising->setMinPreferred(0x06);
        BLEDevice::startAdvertising();
    }

    bool notifyDelta(const DeltaS& ds)
    {
        if (!connected || !delta_char) {
            return false;
        }

        data_deltaS msg = data_deltaS_init_zero;
        msg.timeFrom = ds.time_from_us;
        msg.timeTo = ds.time_to_us;
        msg.deltaX = ds.dx_m;
        msg.deltaY = ds.dy_m;

        uint8_t buf[64];
        pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
        if (!pb_encode(&stream, data_deltaS_fields, &msg)) {
            return false;
        }

        delta_char->setValue(buf, stream.bytes_written);
        delta_char->notify();
        return true;
    }

    void loop()
    {
        uint8_t delta_idx = global_context.active_delta_idx.load(std::memory_order_acquire);
        if (delta_idx != last_delta_idx) {
            last_delta_idx = delta_idx;
            notifyDelta(global_context.delta_buffer[delta_idx]);
        }
    }

    ~Bluetooth() = default;
};