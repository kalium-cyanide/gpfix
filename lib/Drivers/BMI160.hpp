#pragma once

#include "Config.hpp"
#include "Data.hpp"
#include "I2CBus.hpp"
#include <Preferences.h>
#include <esp_timer.h>

// ── Драйвер BMI160 (6-осевой IMU) ───────────────────────────
//
//  Все операции I2C защищены мьютексом I2CBus::Lock.
//  Wire.begin() НЕ вызывается — это делает I2CBus::init().

struct BMI160 {

    // ── Регистры ─────────────────────────────────────────────
    static constexpr uint8_t REG_CHIP_ID         = 0x00;
    static constexpr uint8_t REG_DATA_GYR_X_LSB  = 0x0C;
    static constexpr uint8_t REG_ACC_CONF        = 0x40;
    static constexpr uint8_t REG_ACC_RANGE       = 0x41;
    static constexpr uint8_t REG_GYR_CONF        = 0x42;
    static constexpr uint8_t REG_GYR_RANGE       = 0x43;
    static constexpr uint8_t REG_CMD             = 0x7E;

    static constexpr uint8_t CMD_SOFT_RESET  = 0xB6;
    static constexpr uint8_t CMD_ACC_NORMAL  = 0x11;
    static constexpr uint8_t CMD_GYR_NORMAL  = 0x15;
    static constexpr uint8_t CHIP_ID_EXPECTED = 0xD1;

    // ── Калибровка ───────────────────────────────────────────
    float gx_off = 0.0f, gy_off = 0.0f, gz_off = 0.0f;
    bool  calibrated = false;

    BMI160() { loadCalibration(); }

    bool begin()
    {
        if (readReg(REG_CHIP_ID) != CHIP_ID_EXPECTED) {
            return false;
        }

        writeReg(REG_CMD, CMD_SOFT_RESET);
        delay(100);

        writeReg(REG_CMD, CMD_ACC_NORMAL);
        delay(50);
        writeReg(REG_CMD, CMD_GYR_NORMAL);
        delay(100);

        writeReg(REG_ACC_CONF,  IMUConfig::ACC_CONF);
        writeReg(REG_ACC_RANGE, IMUConfig::ACC_RANGE);
        writeReg(REG_GYR_CONF,  IMUConfig::GYR_CONF);
        writeReg(REG_GYR_RANGE, IMUConfig::GYR_RANGE);

        delay(50);
        return true;
    }

    void calibrate()
    {
        int32_t sgx = 0, sgy = 0, sgz = 0;

        for (uint32_t i = 0; i < IMUConfig::CALIBRATION_SAMPLES; i++) {
            uint8_t buf[12];
            burstRead(REG_DATA_GYR_X_LSB, buf, 12);

            sgx += static_cast<int16_t>(buf[0] | (buf[1] << 8));
            sgy += static_cast<int16_t>(buf[2] | (buf[3] << 8));
            sgz += static_cast<int16_t>(buf[4] | (buf[5] << 8));
            delay(2);
        }

        float n = static_cast<float>(IMUConfig::CALIBRATION_SAMPLES);
        gx_off = (sgx / n) * IMUConfig::GYRO_SCALE;
        gy_off = (sgy / n) * IMUConfig::GYRO_SCALE;
        gz_off = (sgz / n) * IMUConfig::GYRO_SCALE;

        calibrated = true;
        saveCalibration();
    }

    IMUData read()
    {
        uint8_t buf[12];
        burstRead(REG_DATA_GYR_X_LSB, buf, 12);

        int16_t gx = static_cast<int16_t>(buf[0]  | (buf[1]  << 8));
        int16_t gy = static_cast<int16_t>(buf[2]  | (buf[3]  << 8));
        int16_t gz = static_cast<int16_t>(buf[4]  | (buf[5]  << 8));
        int16_t ax = static_cast<int16_t>(buf[6]  | (buf[7]  << 8));
        int16_t ay = static_cast<int16_t>(buf[8]  | (buf[9]  << 8));
        int16_t az = static_cast<int16_t>(buf[10] | (buf[11] << 8));

        IMUData d;
        d.acc_x  = ax * IMUConfig::ACCEL_SCALE;
        d.acc_y  = ay * IMUConfig::ACCEL_SCALE;
        d.acc_z  = az * IMUConfig::ACCEL_SCALE;
        d.gyro_x = gx * IMUConfig::GYRO_SCALE - gx_off;
        d.gyro_y = gy * IMUConfig::GYRO_SCALE - gy_off;
        d.gyro_z = gz * IMUConfig::GYRO_SCALE - gz_off;
        d.timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
        return d;
    }

    bool isCalibrated() const { return calibrated; }

private:
    void writeReg(uint8_t reg, uint8_t val)
    {
        I2CBus::Lock lock;
        Wire.beginTransmission(IMUConfig::BMI160_ADDR);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    uint8_t readReg(uint8_t reg)
    {
        I2CBus::Lock lock;
        Wire.beginTransmission(IMUConfig::BMI160_ADDR);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(IMUConfig::BMI160_ADDR, static_cast<uint8_t>(1));
        return Wire.read();
    }

    void burstRead(uint8_t reg, uint8_t* buf, uint8_t len)
    {
        I2CBus::Lock lock;
        Wire.beginTransmission(IMUConfig::BMI160_ADDR);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(IMUConfig::BMI160_ADDR, len);
        for (uint8_t i = 0; i < len; i++) {
            buf[i] = Wire.read();
        }
    }

    void saveCalibration()
    {
        Preferences prefs;
        prefs.begin("imu_cal", false);
        prefs.putFloat("gx", gx_off);
        prefs.putFloat("gy", gy_off);
        prefs.putFloat("gz", gz_off);
        prefs.end();
    }

    void loadCalibration()
    {
        Preferences prefs;
        if (prefs.begin("imu_cal", true)) {
            gx_off = prefs.getFloat("gx", 0.0f);
            gy_off = prefs.getFloat("gy", 0.0f);
            gz_off = prefs.getFloat("gz", 0.0f);
            prefs.end();
            calibrated = true;
        }
    }
};
