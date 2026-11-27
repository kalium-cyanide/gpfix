#pragma once

#include "BMI160.hpp"
#include "Config.hpp"
#include "Data.hpp"
#include "IModel.hpp"
#include <MadgwickAHRS.h>
#include <esp_timer.h>

// ── Трекер: главный цикл слияния данных IMU + Speed + GPS ────

struct PT1Filter {
    float state = 0, alpha = 1;

    void init(float dt, float cutoff)
    {
        if (cutoff > 0) {
            float rc = 1.0f / (2.0f * static_cast<float>(M_PI) * cutoff);
            alpha = dt / (rc + dt);
        }
    }

    float apply(float in) { state += alpha * (in - state); return state; }
};

struct Tracker {
    BMI160   imu;
    IModel*  model;
    Madgwick filter;

    float    acc_dx = 0, acc_dy = 0;
    uint64_t window_start = 0;
    uint64_t window_us    = BTConfig::NOTIFY_INTERVAL_MS * 1000ULL;
    uint8_t  last_gps_idx   = 255;
    uint8_t  last_speed_idx = 255;

    Tracker(IModel* m, float frequency) : model(m)
    {
        if (imu.begin()) {
            imu.calibrate();
        } else {
            ESP_LOGW("TRACKER", "BMI160 not ready on startup — will retry in background");
        }
        filter.begin(frequency);
        window_start = static_cast<uint64_t>(esp_timer_get_time());
    }

    void loop()
    {
        if (!imu.isAvailable()) {
            static uint32_t retry_counter = 0;
            if (++retry_counter % static_cast<uint32_t>(IMUConfig::SAMPLE_RATE_HZ) == 0) {
                if (imu.begin()) {
                    imu.calibrate();
                }
            }
        }

        IMUData d = imu.read();

        if (imu.isAvailable()) {
            filter.updateIMU(d.gyro_x, d.gyro_y, d.gyro_z,
                             d.acc_x,  d.acc_y,  d.acc_z);
        }

        // Скорость → модель
        uint8_t si = global_context.active_speed_idx.load(std::memory_order_acquire);
        if (si != last_speed_idx) {
            last_speed_idx = si;
            model->onSpeed(global_context.speed_buffer[si]);
        }

        // GPS → модель
        uint8_t gi = global_context.active_gps_idx.load(std::memory_order_acquire);
        if (gi != last_gps_idx) {
            last_gps_idx = gi;
            GPSData gps = global_context.gps_buffer[gi];
            if (gps.valid) model->onGPS(gps);
        }

        DeltaS ds = model->update(d, filter.getYaw());
        acc_dx += ds.dx_m;
        acc_dy += ds.dy_m;

        uint64_t now = d.timestamp_us;
        if (now - window_start >= window_us) {
            DeltaS out {};
            out.time_from_us = window_start;
            out.time_to_us   = now;
            out.dx_m = acc_dx;
            out.dy_m = acc_dy;
            out.yaw  = ds.yaw;

            uint8_t idx = global_context.active_delta_idx.load(std::memory_order_acquire) ^ 1;
            global_context.delta_buffer[idx] = out;
            global_context.active_delta_idx.store(idx, std::memory_order_release);

            acc_dx = acc_dy = 0;
            window_start = now;
        }
    }
};
