#pragma once

#include "config.hpp"
#include "data.hpp"
#include "init.hpp"
#include <MPU6050.h>
#include <Preferences.h>
#include <Wire.h>

struct IMU {
    MPU6050 mpu;

    float ax_off = 0.0f;
    float ay_off = 0.0f;
    float az_off = 0.0f;
    float gx_off = 0.0f;
    float gy_off = 0.0f;
    float gz_off = 0.0f;
    bool calibrated = false;

    IMU()
        : mpu(initMPU())
    {
        loadCalibration();
    }

    bool begin()
    {
        return mpu.testConnection();
    }

    void calibrate()
    {
        int32_t sax = 0, say = 0, saz = 0;
        int32_t sgx = 0, sgy = 0, sgz = 0;
        int16_t ax, ay, az, gx, gy, gz;

        for (uint32_t i = 0; i < IMUConfig::CALIBRATION_SAMPLES; i++) {
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            sax += ax;
            say += ay;
            saz += az;
            sgx += gx;
            sgy += gy;
            sgz += gz;
            delay(2);
        }

        float n = static_cast<float>(IMUConfig::CALIBRATION_SAMPLES);
        ax_off = (sax / n) * IMUConfig::ACCEL_SCALE;
        ay_off = (say / n) * IMUConfig::ACCEL_SCALE;
        az_off = (saz / n) * IMUConfig::ACCEL_SCALE - 9.81f;
        gx_off = (sgx / n) * IMUConfig::GYRO_SCALE;
        gy_off = (sgy / n) * IMUConfig::GYRO_SCALE;
        gz_off = (sgz / n) * IMUConfig::GYRO_SCALE;

        calibrated = true;
        saveCalibration();
    }

    void saveCalibration()
    {
        Preferences prefs;
        prefs.begin("imu_cal", false);
        prefs.putFloat("ax", ax_off);
        prefs.putFloat("ay", ay_off);
        prefs.putFloat("az", az_off);
        prefs.putFloat("gx", gx_off);
        prefs.putFloat("gy", gy_off);
        prefs.putFloat("gz", gz_off);
        prefs.end();
    }

    void loadCalibration()
    {
        Preferences prefs;
        if (prefs.begin("imu_cal", true)) {
            ax_off = prefs.getFloat("ax", 0.0f);
            ay_off = prefs.getFloat("ay", 0.0f);
            az_off = prefs.getFloat("az", 0.0f);
            gx_off = prefs.getFloat("gx", 0.0f);
            gy_off = prefs.getFloat("gy", 0.0f);
            gz_off = prefs.getFloat("gz", 0.0f);
            prefs.end();
            calibrated = true;
        }
    }

    IMUData read()
    {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        IMUData d;
        d.acc_x = ax * IMUConfig::ACCEL_SCALE - ax_off;
        d.acc_y = ay * IMUConfig::ACCEL_SCALE - ay_off;
        d.acc_z = az * IMUConfig::ACCEL_SCALE - az_off;
        d.gyro_x = gx * IMUConfig::GYRO_SCALE - gx_off;
        d.gyro_y = gy * IMUConfig::GYRO_SCALE - gy_off;
        d.gyro_z = gz * IMUConfig::GYRO_SCALE - gz_off;
        d.timestamp_us = static_cast<uint64_t>(esp_timer_get_time());

        return d;
    }

    bool isCalibrated() const
    {
        return calibrated;
    }

    ~IMU() = default;
};
