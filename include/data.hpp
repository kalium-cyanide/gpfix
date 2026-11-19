#pragma once

#include <atomic>
#include <cstdint>

#pragma pack(push, 1)

struct IMURaw {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
};

struct IMUData {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint64_t timestamp_us;
};

struct SpeedData {
    float speed_kmh;
    uint64_t timestamp_us;
    bool valid;
};

using CANData = SpeedData; // Совместимость со старым кодом

struct GPSData {
    double lat;
    double lon;
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
    DeltaS delta_buffer[2];
    GPSData gps_buffer[2];
    SpeedData speed_buffer[2];
    #define can_buffer speed_buffer

    std::atomic<uint8_t> active_delta_idx { 0 };
    std::atomic<uint8_t> active_gps_idx { 0 };
    std::atomic<uint8_t> active_speed_idx { 0 };
    #define active_can_idx active_speed_idx
};

extern Context global_context;