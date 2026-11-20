#pragma once

#include "Config.hpp"
#include "IModel.hpp"
#include <Arduino.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <atomic>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// ── Бинарный логгер на SD-карту ──────────────────────────────

enum LogType : uint8_t { LOG_IMU = 0x01, LOG_CAN = 0x02, LOG_GPS = 0x03 };

#pragma pack(push, 1)

struct LogHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t imu_hz;
    uint64_t start_time_us;
    float    accel_scale;
    float    gyro_scale;
    uint8_t  acc_range;
    uint8_t  gyr_range;
    uint8_t  _pad[2];
};

struct LogEntry {
    uint8_t  type;
    uint64_t timestamp_us;
    union {
        struct { float ax, ay, az, gx, gy, gz, yaw; } imu;
        struct { float speed_kmh; } can;
        struct { double lat, lon; } gps;
    };
};

#pragma pack(pop)

static_assert(sizeof(LogHeader) == 28, "LogHeader must be 28 bytes");
static_assert(sizeof(LogEntry)  == 37, "LogEntry must be 37 bytes");

struct LogModel : IModel {

    QueueHandle_t     log_queue = nullptr;
    SemaphoreHandle_t done_sem  = nullptr;
    File              log_file;
    SPIClass          sd_spi;

    std::atomic<bool>     sd_ready { false };
    std::atomic<bool>     running  { false };
    std::atomic<uint32_t> records_dropped { 0 };

    uint32_t records_written  = 0;
    uint32_t last_flush_count = 0;
    uint64_t bytes_written    = 0;
    uint64_t free_at_start    = 0;
    bool     writer_started   = false;
    bool     write_failed     = false;

    LogModel() : sd_spi(HSPI) {}
    LogModel(const LogModel&) = delete;
    LogModel& operator=(const LogModel&) = delete;

    bool begin()
    {
        sd_spi.begin(PinConfig::SD_SCK, PinConfig::SD_MISO,
                     PinConfig::SD_MOSI, PinConfig::SD_CS);

        if (!SD.begin(PinConfig::SD_CS, sd_spi, SDConfig::SPI_FREQ)) {
            ESP_LOGE("SDLOG", "SD init failed");
            return false;
        }

        uint32_t session = 0;
        char filename[32];
        do {
            session = nextSession();
            snprintf(filename, sizeof(filename), "/log_%04u.bin",
                     static_cast<unsigned>(session));
        } while (SD.exists(filename));

        log_file = SD.open(filename, FILE_WRITE);
        if (!log_file) { ESP_LOGE("SDLOG", "Cannot create %s", filename); return false; }

        LogHeader hdr {};
        hdr.magic         = 0x58465047;
        hdr.version       = 1;
        hdr.imu_hz        = static_cast<uint16_t>(IMUConfig::SAMPLE_RATE_HZ);
        hdr.start_time_us = static_cast<uint64_t>(esp_timer_get_time());
        hdr.accel_scale   = IMUConfig::ACCEL_SCALE;
        hdr.gyro_scale    = IMUConfig::GYRO_SCALE;
        hdr.acc_range     = IMUConfig::ACC_RANGE;
        hdr.gyr_range     = IMUConfig::GYR_RANGE;

        if (log_file.write(reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr))
                != sizeof(hdr)) {
            log_file.close();
            return false;
        }
        log_file.flush();

        free_at_start = SD.totalBytes() - SD.usedBytes();
        log_queue = xQueueCreate(SDConfig::QUEUE_SIZE, sizeof(LogEntry));
        done_sem  = xSemaphoreCreateBinary();
        if (!log_queue || !done_sem) { cleanupRtos(); log_file.close(); return false; }

        sd_ready = true;
        running  = true;

        if (xTaskCreatePinnedToCore(writerTaskFunc, "sd_writer",
                SDConfig::WRITER_STACK, this, SDConfig::WRITER_PRIO,
                nullptr, SDConfig::WRITER_CORE) != pdPASS) {
            sd_ready = false; running = false;
            cleanupRtos(); log_file.close();
            return false;
        }
        writer_started = true;

        ESP_LOGI("SDLOG", "Session %u -> %s  queue=%u",
                 static_cast<unsigned>(session), filename,
                 static_cast<unsigned>(SDConfig::QUEUE_SIZE));
        return true;
    }

    bool isReady() const { return sd_ready.load(std::memory_order_acquire); }

    DeltaS update(const IMUData& imu, float yaw) override
    {
        if (isReady()) {
            LogEntry e {};
            e.type = LOG_IMU;
            e.timestamp_us = imu.timestamp_us;
            e.imu = { imu.acc_x, imu.acc_y, imu.acc_z,
                       imu.gyro_x, imu.gyro_y, imu.gyro_z, yaw };
            enqueue(e);
        }
        DeltaS ds {};
        ds.time_from_us = ds.time_to_us = imu.timestamp_us;
        ds.yaw = yaw;
        return ds;
    }

    void onSpeed(const SpeedData& s) override
    {
        if (isReady() && s.valid) {
            LogEntry e {};
            e.type = LOG_CAN;
            e.timestamp_us = s.timestamp_us;
            e.can.speed_kmh = s.speed_kmh;
            enqueue(e);
        }
    }

    void onGPS(const GPSData& g) override
    {
        if (isReady() && g.valid) {
            LogEntry e {};
            e.type = LOG_GPS;
            e.timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
            e.gps = { g.lat, g.lon };
            enqueue(e);
        }
    }

    void end()
    {
        sd_ready = false;
        if (writer_started) {
            running = false;
            xSemaphoreTake(done_sem, pdMS_TO_TICKS(3000));
            writer_started = false;
        }
        cleanupRtos();
    }

    ~LogModel() override { end(); }

private:
    void enqueue(const LogEntry& e)
    {
        if (xQueueSend(log_queue, &e, 0) != pdTRUE)
            records_dropped.fetch_add(1, std::memory_order_relaxed);
    }

    void cleanupRtos()
    {
        if (log_queue) { vQueueDelete(log_queue); log_queue = nullptr; }
        if (done_sem)  { vSemaphoreDelete(done_sem); done_sem = nullptr; }
    }

    uint32_t nextSession()
    {
        Preferences p;
        p.begin("sdlog", false);
        uint32_t n = p.getULong("session", 0) + 1;
        p.putULong("session", n);
        p.end();
        return n;
    }

    size_t drainOnce(LogEntry* batch, TickType_t wait)
    {
        if (xQueueReceive(log_queue, &batch[0], wait) != pdTRUE) return 0;
        size_t count = 1;
        while (count < SDConfig::WRITE_BATCH
               && xQueueReceive(log_queue, &batch[count], 0) == pdTRUE)
            count++;

        size_t bytes = sizeof(LogEntry) * count;
        size_t written = log_file.write(reinterpret_cast<const uint8_t*>(batch), bytes);
        if (written != bytes) {
            write_failed = true; sd_ready = false; running = false;
        }
        bytes_written   += written;
        records_written += static_cast<uint32_t>(written / sizeof(LogEntry));
        return count;
    }

    void periodicFlush()
    {
        if (records_written - last_flush_count >= SDConfig::FLUSH_INTERVAL) {
            log_file.flush();
            last_flush_count = records_written;
            if (bytes_written + SDConfig::MIN_FREE_BYTES >= free_at_start) {
                ESP_LOGE("SDLOG", "SD nearly full — stopping");
                sd_ready = false; running = false;
            }
        }
    }

    void writerLoop()
    {
        LogEntry batch[SDConfig::WRITE_BATCH];
        while (running.load(std::memory_order_acquire)) {
            drainOnce(batch, pdMS_TO_TICKS(1000));
            periodicFlush();
        }
        while (!write_failed && drainOnce(batch, 0) > 0) {}
        log_file.flush(); log_file.close();
        ESP_LOGI("SDLOG", "Closed: %u written, %u dropped",
                 static_cast<unsigned>(records_written),
                 static_cast<unsigned>(records_dropped.load()));
        xSemaphoreGive(done_sem);
        vTaskDelete(nullptr);
    }

    static void writerTaskFunc(void* p) { static_cast<LogModel*>(p)->writerLoop(); }
};
