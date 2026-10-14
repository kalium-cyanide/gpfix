#pragma once

#include "config.hpp"
#include "data.hpp"
#include "esp_timer.h"
#include "imu.hpp"
#include "model.hpp"
#include <MadgwickAHRS.h>

struct PT1Filter {
    float state = 0.0f;
    float alpha = 1.0f;

    void init(float dt, float cutoff_freq)
    {
        if (cutoff_freq > 0.0f) {
            float rc = 1.0f / (2.0f * static_cast<float>(M_PI) * cutoff_freq);
            alpha = dt / (rc + dt);
        } else {
            alpha = 1.0f;
        }
    }

    float apply(float input)
    {
        state += alpha * (input - state);
        return state;
    }
};

struct Tracker {
    IMU imu;
    Model model;
    Madgwick filter;

    float acc_dx = 0.0f;
    float acc_dy = 0.0f;
    uint64_t window_start = 0;
    uint64_t window_us = BTConfig::NOTIFY_INTERVAL_MS * 1000ULL;
    uint8_t last_gps_idx = 255;

    Tracker(float frequency)
    {
        imu.begin();
        imu.calibrate();
        filter.begin(frequency);
        window_start = static_cast<uint64_t>(esp_timer_get_time());
    }

    void loop()
    {
        IMUData imu_data = imu.read();

        filter.updateIMU(
            imu_data.gyro_x,
            imu_data.gyro_y,
            imu_data.gyro_z,
            imu_data.acc_x,
            imu_data.acc_y,
            imu_data.acc_z
        );

        CANData can {};
        can.speed_kmh = 0.0f;
        can.timestamp_us = imu_data.timestamp_us;
        can.valid = false;

        uint8_t gps_idx = global_context.active_gps_idx.load(std::memory_order_acquire);
        if (gps_idx != last_gps_idx) {
            last_gps_idx = gps_idx;
            GPSData gps = global_context.gps_buffer[gps_idx];
            if (gps.valid) {
                model.calibrate(gps, imu_data, can);
            }
        }

        model.resetHeading(filter.getYaw());

        DeltaS ds = model.update(imu_data, can);
        acc_dx += ds.dx_m;
        acc_dy += ds.dy_m;

        uint64_t now = imu_data.timestamp_us;
        if (now - window_start >= window_us) {
            DeltaS out {};
            out.time_from_us = window_start;
            out.time_to_us = now;
            out.dx_m = acc_dx;
            out.dy_m = acc_dy;
            out.heading_deg = model.heading();

            uint8_t next_delta_idx = global_context.active_delta_idx.load(std::memory_order_acquire) ^ 1;
            global_context.delta_buffer[next_delta_idx] = out;
            global_context.active_delta_idx.store(next_delta_idx, std::memory_order_release);

            acc_dx = 0.0f;
            acc_dy = 0.0f;
            window_start = now;
        }
    }

    ~Tracker() = default;
};

