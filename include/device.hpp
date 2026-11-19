#pragma once

#include "bicycle_eskf.hpp"
#include "speed_source.hpp"

#if defined(DEVICE_BIKE)
    #include "hall_speed.hpp"

    // Конфигурация для велосипеда (датчик Холла на колесе)
    struct BikeDevice {
        HallSpeed speed_sensor;

        const char* name() const { return "Bike"; }

        ISpeedSource* getSpeedSource() { return &speed_sensor; }

        ESKFParams getModelParams() const {
            ESKFParams p;
            p.wheelbase       = 1.05f;  // База велосипеда (~1.05 м)
            p.steer_ratio     = 1.0f;   // Руль велосипеда (прямое управление 1:1)
            p.c_steer         = 1.0f / 1.05f;
            p.r_obd_var       = 0.05f;  // Обороты колеса точнее дискретности OBD 1 км/ч
            p.gps_default_acc = 3.5f;   // Точность GPS под открытым небом
            p.chi2_gate       = 9.21f;  // 2-DOF 99%
            return p;
        }
    };

    using CurrentDevice = BikeDevice;

#else // DEVICE_CAR (по умолчанию)
    #include "obd2.hpp"

    // Конфигурация для автомобиля (CAN / OBD2)
    struct CarDevice {
        OBD2 speed_sensor;

        const char* name() const { return "Car"; }

        ISpeedSource* getSpeedSource() { return &speed_sensor; }

        ESKFParams getModelParams() const {
            ESKFParams p;
            p.wheelbase       = 2.6f;   // База автомобиля (~2.6-2.8 м)
            p.steer_ratio     = 15.0f;  // Передаточное отношение руля
            p.c_steer         = 1.0f / (2.6f * 15.0f);
            p.r_obd_var       = 0.25f;  // Дисперсия скорости OBD2 (~0.5 м/с)
            p.gps_default_acc = 5.0f;   // Оценка погрешности GPS
            p.chi2_gate       = 9.21f;  // 2-DOF 99%
            return p;
        }
    };

    using CurrentDevice = CarDevice;

#endif
