#pragma once

#include "Data.hpp"

// ── Общий интерфейс датчика скорости ─────────────────────────
//  OBD2       — автомобиль (CAN / TWAI)
//  HallSpeed  — велосипед  (датчик Холла на колесе)

struct ISpeedSource {
    virtual ~ISpeedSource() = default;
    virtual bool begin() = 0;
    virtual SpeedData readSpeed() = 0;
};
