#pragma once

#include "BicycleESKF.hpp"
#include "Config.hpp"
#include "CycplusSpeed.hpp"
#include "HallSpeed.hpp"

// ── Профиль устройства: Велосипед ────────────────────────────

struct BikeDevice {
    CycplusSpeed ble_speed_sensor;
    HallSpeed    hall_speed_sensor;

    const char* name() const { return "Bike"; }

    ISpeedSource* getSpeedSource()
    {
        return BikeConfig::USE_BLE_SPEED
            ? static_cast<ISpeedSource*>(&ble_speed_sensor)
            : static_cast<ISpeedSource*>(&hall_speed_sensor);
    }

    ESKFParams getModelParams() const
    {
        ESKFParams p;
        p.wheelbase       = 1.05f;
        p.steer_ratio     = 1.0f;
        p.c_steer         = 1.0f / 1.05f;
        p.r_obd_var       = 0.05f;
        p.gps_default_acc = 3.5f;
        p.chi2_gate       = 9.21f;
        return p;
    }
};
