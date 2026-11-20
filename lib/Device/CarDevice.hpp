#pragma once

#include "BicycleESKF.hpp"
#include "OBD2.hpp"

// ── Профиль устройства: Автомобиль ───────────────────────────

struct CarDevice {
    OBD2 speed_sensor;

    const char* name() const { return "Car"; }

    ISpeedSource* getSpeedSource() { return &speed_sensor; }

    ESKFParams getModelParams() const
    {
        ESKFParams p;
        p.wheelbase       = 2.6f;
        p.steer_ratio     = 15.0f;
        p.c_steer         = 1.0f / (2.6f * 15.0f);
        p.r_obd_var       = 0.25f;
        p.gps_default_acc = 5.0f;
        p.chi2_gate       = 9.21f;
        return p;
    }
};
