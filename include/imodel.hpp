#pragma once

#include "data.hpp"

struct IModel {
  virtual ~IModel() = default;

  virtual DeltaS update(const IMUData &imu, float yaw) = 0;

  virtual void onCAN(const CANData &can) = 0;

  virtual void onGPS(const GPSData &gps) = 0;
};
