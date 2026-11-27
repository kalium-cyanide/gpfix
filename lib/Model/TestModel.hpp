#pragma once

#include "Config.hpp"
#include "Data.hpp"
#include "IModel.hpp"
#include <ArduinoEigen/Eigen/Core>
#include <cmath>

// ── Тестовая модель (настольная диагностика без транспорта) ───
//
//  1. BMI160: модуль ускорения |a|
//  2. Автокалибровка вертикали u_z в покое
//  3. Проекция гироскопа → yaw, интегрирование курса
//  4. Датчик Холла: импульсы и скорость
//  5. GPS от смартфона через BLE

struct TestModel : public IModel {

    // Данные для OLED-дисплея
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float a_norm   = 0;
    float w_proj   = 0;
    float current_yaw_deg = 0;

    // Вектор вертикали (auto-leveling)
    Eigen::Vector3f u_z = Eigen::Vector3f(0, 0, 1);
    Eigen::Vector3f acc_accum = Eigen::Vector3f::Zero();
    bool     is_leveled    = false;
    uint32_t level_samples = 0;

    // Скорость
    float    speed_kmh  = 0;
    uint32_t pulse_count = 0;
    bool     speed_valid = false;

    // GPS
    double   gps_lat = 0, gps_lon = 0;
    bool     gps_valid = false;
    uint32_t gps_fixes = 0;

    // Таймер
    uint32_t imu_updates = 0;
    uint64_t last_ts_us  = 0;

    DeltaS update(const IMUData& imu, float /*yaw*/) override
    {
        imu_updates++;
        ax = imu.acc_x; ay = imu.acc_y; az = imu.acc_z;
        gx = imu.gyro_x; gy = imu.gyro_y; gz = imu.gyro_z;
        a_norm = std::sqrt(ax*ax + ay*ay + az*az);

        // 1. Автокалибровка вертикали (только при реалистичной гравитации 6..14 м/с²)
        if (a_norm > 6.0f && a_norm < 14.0f) {
            if (!is_leveled || speed_kmh < 0.2f) {
                acc_accum += Eigen::Vector3f(ax, ay, az);
                level_samples++;
                if (level_samples >= 50) {
                    float norm = acc_accum.norm();
                    if (norm > 50.0f * 6.0f) {
                        u_z = acc_accum / norm;
                        is_leveled = true;
                    }
                }
            }
        }

        // 2. Проекция 3D гироскопа на вертикаль
        w_proj = is_leveled
            ? (gx * u_z(0) + gy * u_z(1) + gz * u_z(2))
            : gz;

        // 3. Интегрирование yaw
        float dt = 0;
        if (last_ts_us != 0) {
            dt = static_cast<float>(imu.timestamp_us - last_ts_us) * 1e-6f;
            if (dt > 0 && dt < 0.5f) {
                current_yaw_deg += w_proj * dt;
                if (current_yaw_deg >  180.0f) current_yaw_deg -= 360.0f;
                if (current_yaw_deg < -180.0f) current_yaw_deg += 360.0f;
            }
        }
        last_ts_us = imu.timestamp_us;

        DeltaS ds {};
        ds.time_from_us = imu.timestamp_us;
        ds.time_to_us   = imu.timestamp_us;
        ds.yaw = current_yaw_deg * 3.14159265f / 180.0f;
        return ds;
    }

    void onSpeed(const SpeedData& s) override
    {
        speed_kmh  = s.speed_kmh;
        speed_valid = s.valid;
        if (s.valid && s.speed_kmh > 0.05f) pulse_count++;
    }

    void onGPS(const GPSData& g) override
    {
        if (g.valid) {
            gps_lat = g.lat;  gps_lon = g.lon;
            gps_valid = true; gps_fixes++;
        }
    }
};
