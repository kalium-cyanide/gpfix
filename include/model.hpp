#pragma once

#include "config.hpp"
#include "data.hpp"
#include <cmath>

struct ModelParams {
    float velocity_scale = 1.0f;
    float accel_bias_x = 0.0f;
    float accel_bias_y = 0.0f;
    float accel_bias_z = 0.0f;
    float gyro_bias_z = 0.0f;
    float heading_offset = 0.0f;
    float steering_ratio = 1.0f;
    float wheelbase_m = 2.7f;
};

struct Model {
    ModelParams params;
    float heading_deg = 0.0f;
    float velocity_mps = 0.0f;
    uint64_t last_update_us = 0;
    bool initialized = false;

    Model() = default;
    Model(const ModelParams& p)
        : params(p)
    {
    }

    DeltaS update(const IMUData& imu, const CANData& can)
    {
        uint64_t now = imu.timestamp_us;
        DeltaS ds {};
        ds.time_from_us = last_update_us;
        ds.time_to_us = now;

        if (!initialized) {
            initialized = true;
            last_update_us = now;
            return ds;
        }

        ds.dx_m = 0.0f;
        ds.dy_m = 0.0f;
        ds.heading_deg = heading_deg;

        last_update_us = now;
        return ds;
    }

    void calibrate(const GPSData& gps, const IMUData& imu, const CANData& can)
    {
        if (!gps.valid) {
            return;
        }
    }

    void resetHeading(float gps_heading_deg)
    {
        heading_deg = gps_heading_deg;
    }

    float heading() const
    {
        return heading_deg;
    }

    float velocity() const
    {
        return velocity_mps;
    }

    bool isInitialized() const
    {
        return initialized;
    }

    ~Model() = default;
};
