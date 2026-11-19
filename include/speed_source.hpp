#pragma once

#include "data.hpp"

// Общий интерфейс датчика скорости (OBD2 для автомобиля или датчик Холла для велосипеда)
struct ISpeedSource {
    virtual ~ISpeedSource() = default;
    virtual bool begin() = 0;
    virtual SpeedData readSpeed() = 0;
};
