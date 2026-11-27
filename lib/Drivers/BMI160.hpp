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

    // ── Калибровка и состояние ──────────────────────────────
    float gx_off = 0.0f, gy_off = 0.0f, gz_off = 0.0f;
    bool  calibrated = false;
    bool  available  = false;
    uint8_t i2c_addr = IMUConfig::BMI160_ADDR;

    BMI160() { loadCalibration(); }

    bool begin()
    {
        // 1. Автоопределение I2C адреса (0x68 при SDO->GND, 0x69 при SDO->3.3V)
        uint8_t found = 0;
        for (uint8_t a : { static_cast<uint8_t>(0x68), static_cast<uint8_t>(0x69) }) {
            uint8_t id = readRegFrom(a, REG_CHIP_ID);
            if (id == CHIP_ID_EXPECTED) {
                found = a;
                break;
            }
        }

        if (found == 0) {
            uint8_t id68 = readRegFrom(0x68, REG_CHIP_ID);
            uint8_t id69 = readRegFrom(0x69, REG_CHIP_ID);
            ESP_LOGE("BMI160", "BMI160 NOT RESPONDING! (at 0x68 CHIP_ID=0x%02X, at 0x69 CHIP_ID=0x%02X, expected 0xD1)",
                     id68, id69);
            ESP_LOGE("BMI160", "Check: 1) CS pin connected to 3.3V (REQUIRED for I2C!), 2) VCC/GND, 3) SDA/SCL wiring");
            available = false;
            return false;
        }

        i2c_addr = found;
        available = true;
        ESP_LOGI("BMI160", "BMI160 detected at address 0x%02X (CHIP_ID=0x%02X)", i2c_addr, CHIP_ID_EXPECTED);

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
        if (!available) return;

        int32_t sgx = 0, sgy = 0, sgz = 0;

        for (uint32_t i = 0; i < IMUConfig::CALIBRATION_SAMPLES; i++) {
            uint8_t buf[12];
            if (burstRead(REG_DATA_GYR_X_LSB, buf, 12)) {
                sgx += static_cast<int16_t>(buf[0] | (buf[1] << 8));
                sgy += static_cast<int16_t>(buf[2] | (buf[3] << 8));
                sgz += static_cast<int16_t>(buf[4] | (buf[5] << 8));
            }
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
        IMUData d {};
        d.timestamp_us = static_cast<uint64_t>(esp_timer_get_time());

        if (!available) {
            return d;
        }

        uint8_t buf[12];
        if (!burstRead(REG_DATA_GYR_X_LSB, buf, 12)) {
            return d;
        }

        int16_t gx = static_cast<int16_t>(buf[0]  | (buf[1]  << 8));
        int16_t gy = static_cast<int16_t>(buf[2]  | (buf[3]  << 8));
        int16_t gz = static_cast<int16_t>(buf[4]  | (buf[5]  << 8));
        int16_t ax = static_cast<int16_t>(buf[6]  | (buf[7]  << 8));
        int16_t ay = static_cast<int16_t>(buf[8]  | (buf[9]  << 8));
        int16_t az = static_cast<int16_t>(buf[10] | (buf[11] << 8));

        d.acc_x  = ax * IMUConfig::ACCEL_SCALE;
        d.acc_y  = ay * IMUConfig::ACCEL_SCALE;
        d.acc_z  = az * IMUConfig::ACCEL_SCALE;
        d.gyro_x = gx * IMUConfig::GYRO_SCALE - gx_off;
        d.gyro_y = gy * IMUConfig::GYRO_SCALE - gy_off;
        d.gyro_z = gz * IMUConfig::GYRO_SCALE - gz_off;
        return d;
    }

    bool isCalibrated() const { return calibrated; }
    bool isAvailable()  const { return available; }

private:
    void writeReg(uint8_t reg, uint8_t val)
    {
        I2CBus::Lock lock;
        Wire.beginTransmission(i2c_addr);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    uint8_t readRegFrom(uint8_t addr, uint8_t reg)
    {
        I2CBus::Lock lock;
        Wire.beginTransmission(addr);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) {
            return 0xFF;
        }
        if (Wire.requestFrom(addr, static_cast<uint8_t>(1)) != 1) {
            return 0xFF;
        }
        return Wire.read();
    }

    uint8_t readReg(uint8_t reg)
    {
        return readRegFrom(i2c_addr, reg);
    }

    bool burstRead(uint8_t reg, uint8_t* buf, uint8_t len)
    {
        I2CBus::Lock lock;
        Wire.beginTransmission(i2c_addr);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) {
            memset(buf, 0, len);
            return false;
        }
        if (Wire.requestFrom(i2c_addr, len) != len) {
            memset(buf, 0, len);
            return false;
        }
        for (uint8_t i = 0; i < len; i++) {
            buf[i] = Wire.read();
        }
        return true;
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
