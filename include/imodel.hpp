#pragma once

#include "data.hpp"

struct IModel {
    virtual ~IModel() = default;

    virtual DeltaS update(const IMUData& imu, float yaw) = 0;

    virtual void onSpeed(const SpeedData& speed) = 0;

    virtual void onCAN(const CANData& can) { onSpeed(can); }

    virtual void onGPS(const GPSData& gps) = 0;
};

struct CompositeModel : IModel {
    IModel* primary = nullptr;
    IModel* secondary = nullptr;

    CompositeModel(IModel* prim = nullptr, IModel* sec = nullptr)
        : primary(prim)
        , secondary(sec)
    {
    }

    DeltaS update(const IMUData& imu, float yaw) override
    {
        if (secondary) {
            secondary->update(imu, yaw);
        }
        return primary ? primary->update(imu, yaw) : DeltaS { };
    }

    void onSpeed(const SpeedData& speed) override
    {
        if (primary)
            primary->onSpeed(speed);
        if (secondary)
            secondary->onSpeed(speed);
    }

    void onCAN(const CANData& can) override
    {
        onSpeed(can);
    }

    void onGPS(const GPSData& gps) override
    {
        if (primary)
            primary->onGPS(gps);
        if (secondary)
            secondary->onGPS(gps);
    }
};
