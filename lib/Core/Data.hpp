#pragma once

#include <atomic>
#include <cstdint>

#pragma pack(push, 1)

struct IMURaw {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

struct IMUData {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    uint64_t timestamp_us;
};

struct SpeedData {
    float speed_kmh;
    uint64_t timestamp_us;
    bool valid;
};

using CANData = SpeedData;

struct GPSData {
    double lat, lon;
    bool valid;
};

struct DeltaS {
    uint64_t time_from_us;
    uint64_t time_to_us;
    float dx_m;
    float dy_m;
    float yaw;
};

#pragma pack(pop)

struct Context {
    DeltaS    delta_buffer[2];
    GPSData   gps_buffer[2];
    SpeedData speed_buffer[2];

    std::atomic<uint8_t> active_delta_idx { 0 };
    std::atomic<uint8_t> active_gps_idx   { 0 };
    std::atomic<uint8_t> active_speed_idx { 0 };
    std::atomic<bool>    ble_connected    { false };
};

extern Context global_context;
