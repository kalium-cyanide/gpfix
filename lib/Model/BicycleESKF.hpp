#pragma once
// ────────────────────────────────────────────────────────────────
// BicycleESKF — Error-State Kalman Filter с кинематической моделью
// велосипеда.  Порт model.py → C++ / Eigen (float, 7-state).
//
// Состояние X[7]:
//   0: x       — East  (м)
//   1: y       — North (м)
//   2: theta   — курс  (рад)
//   3: v       — продольная скорость (м/с)
//   4: b_w     — bias гироскопа рыскания (рад/с)
//   5: b_a     — bias продольного акселерометра (м/с²)
//   6: k_slip  — коэфф. масштаба колеса
// ────────────────────────────────────────────────────────────────

#include "Config.hpp"
#include "Data.hpp"
#include "IModel.hpp"

#include <ArduinoEigen/Eigen/Core>
#include <ArduinoEigen/Eigen/LU>
#include <cmath>

// ── Константы ────────────────────────────────────────────────

static constexpr float  PI_F      = 3.14159265358979323846f;
static constexpr float  DEG2RAD_F = PI_F / 180.0f;
static constexpr float  RAD2DEG_F = 180.0f / PI_F;
static constexpr double R_EARTH_D = 6378137.0;

using Vec7f    = Eigen::Matrix<float, 7, 1>;
using Mat7f    = Eigen::Matrix<float, 7, 7>;
using Vec3f    = Eigen::Matrix<float, 3, 1>;
using Vec2f    = Eigen::Matrix<float, 2, 1>;
using Mat2f    = Eigen::Matrix<float, 2, 2>;

// ── Утилиты ──────────────────────────────────────────────────

inline float wrap_angle(float a)
{
    if (!std::isfinite(a)) return NAN;
    a = std::fmod(a + PI_F, 2.0f * PI_F);
    if (a < 0.0f) a += 2.0f * PI_F;
    return a - PI_F;
}

// ── ENU-проекция (East-North-Up) ─────────────────────────────

struct ENU {
    double lat0 = 0, lon0 = 0, kx = 0;
    bool   ready = false;

    void setOrigin(double lat, double lon)
    {
        lat0 = lat; lon0 = lon;
        kx   = R_EARTH_D * std::cos(lat * M_PI / 180.0);
        ready = true;
    }

    void fwd(double lat, double lon, float& xe, float& yn) const
    {
        xe = static_cast<float>((lon - lon0) * (M_PI / 180.0) * kx);
        yn = static_cast<float>((lat - lat0) * (M_PI / 180.0) * R_EARTH_D);
    }
};

// ── Параметры фильтра ────────────────────────────────────────

struct ESKFParams {
    float wheelbase       = 2.6f;
    float steer_ratio     = 15.0f;
    float c_steer         = 1.0f / (2.6f * 15.0f);
    float chi2_gate       = 9.21f;
    float min_gps_sigma   = 2.5f;
    float r_obd_var       = 0.25f;
    int   recover_n       = 3;
    float gps_default_acc = 5.0f;
};

// ══════════════════════════════════════════════════════════════
//  BicycleESKF : IModel
// ══════════════════════════════════════════════════════════════

struct BicycleESKF : IModel {

    Vec7f      X;
    Mat7f      P;
    ESKFParams par;
    ENU        enu;

    // Трекинг
    float    prev_x = 0, prev_y = 0;
    uint64_t last_ts_us = 0;
    bool     heading_init = false;
    bool     is_dr = false;
    int      consecutive_good = 0;
    float    dist_since_fix = 0, accum_wheel_dist = 0;
    float    last_fix_x = 0, last_fix_y = 0, last_fix_t = 0;
    float    current_t = 0;
    bool     has_anchor = false;
    float    anchor_x = 0, anchor_y = 0;

    SpeedData latest_speed {};
    uint64_t  last_speed_ts = 0;

    // 3D-автовыравнивание
    Vec3f    u_z = Vec3f(0.0f, 0.0f, 1.0f);
    bool     is_leveled = false;
    Vec3f    acc_accum = Vec3f::Zero();
    uint32_t level_samples = 0;

    // Автокалибровка продольной оси (куда смотрит "вперёд" велосипеда в СК датчика)
    Vec3f    u_x = Vec3f(1.0f, 0.0f, 0.0f);
    Vec3f    u_y = Vec3f(0.0f, 1.0f, 0.0f);
    bool     is_fwd_calibrated = false;
    Vec3f    fwd_accum = Vec3f::Zero();
    uint32_t fwd_samples = 0;
    float    cal_ref_v = 0.0f;
    uint64_t cal_ref_ts = 0;
    int8_t   cal_accel_sign = 0;

    // ── Constructor ──────────────────────────────────────────

    explicit BicycleESKF(const ESKFParams& p = {}) : par(p)
    {
        X.setZero();
        X(6) = 1.0f;

        P.setZero();
        P(0,0) = 4.0f;   P(1,1) = 4.0f;
        P(2,2) = 0.15f;  P(3,3) = 0.25f;
        P(4,4) = 1e-5f;  P(5,5) = 0.1f;
        P(6,6) = 0.01f;
    }

    // ══════════════════════════════════════════════════════════
    //  IModel
    // ══════════════════════════════════════════════════════════

    DeltaS update(const IMUData& imu, float /*madgwick_yaw*/) override
    {
        if (!is_leveled || (latest_speed.valid && latest_speed.speed_kmh < 0.2f)) {
            updateLeveling(imu.acc_x, imu.acc_y, imu.acc_z);
        }

        float dt = 0.0f;
        if (last_ts_us != 0) {
            dt = static_cast<float>(imu.timestamp_us - last_ts_us) * 1e-6f;
            if (dt <= 0.0f || dt > 0.5f) dt = 0.0f;
        }
        last_ts_us = imu.timestamp_us;

        if (dt > 0.0f) {
            float omega = projectYawRate(imu.gyro_x, imu.gyro_y, imu.gyro_z);
            if (is_leveled && cal_accel_sign != 0 && std::fabs(omega) < 0.15f && fwd_samples < 500) {
                updateForwardAxis(imu.acc_x, imu.acc_y, imu.acc_z, cal_accel_sign);
            }
            float a_meas = is_fwd_calibrated
                ? (imu.acc_x * u_x(0) + imu.acc_y * u_x(1) + imu.acc_z * u_x(2))
                : 0.0f;
            predict(omega, a_meas, dt);
        }

        if (latest_speed.valid && latest_speed.timestamp_us != last_speed_ts) {
            last_speed_ts = latest_speed.timestamp_us;
            float v = latest_speed.speed_kmh / 3.6f;
            (v < 0.05f) ? updateZUPT() : updateOBD(v);
        }

        DeltaS ds {};
        ds.time_from_us = imu.timestamp_us;
        ds.time_to_us   = imu.timestamp_us;
        ds.dx_m = X(0) - prev_x;
        ds.dy_m = X(1) - prev_y;
        ds.yaw  = X(2);
        prev_x = X(0);
        prev_y = X(1);
        return ds;
    }

    void onSpeed(const SpeedData& speed) override
    {
        latest_speed = speed;
        if (!speed.valid) return;

        float v = speed.speed_kmh / 3.6f;
        if (cal_ref_ts == 0) {
            cal_ref_v  = v;
            cal_ref_ts = speed.timestamp_us;
            return;
        }

        float dt_v = static_cast<float>(speed.timestamp_us - cal_ref_ts) * 1e-6f;
        if (dt_v >= 0.25f) {
            float dv = v - cal_ref_v;
            if (dv > 0.35f && v > 0.4f) {
                cal_accel_sign = 1;
            } else if (dv < -0.45f && cal_ref_v > 0.8f) {
                cal_accel_sign = -1;
            } else {
                cal_accel_sign = 0;
            }
            cal_ref_v  = v;
            cal_ref_ts = speed.timestamp_us;
        }
    }

    void onGPS(const GPSData& gps) override
    {
        if (!gps.valid) return;
        if (!enu.ready) enu.setOrigin(gps.lat, gps.lon);

        float gx, gy;
        enu.fwd(gps.lat, gps.lon, gx, gy);
        updateGPS(gx, gy, par.gps_default_acc);
    }

    // ── Diagnostics ──────────────────────────────────────────

    float sigmaPos() const
    {
        return std::hypot(std::sqrt(std::max(0.0f, P(0,0))),
                          std::sqrt(std::max(0.0f, P(1,1))));
    }
    float sigmaThetaDeg() const { return std::sqrt(std::max(0.0f, P(2,2))) * RAD2DEG_F; }
    bool  isDR() const { return is_dr; }

private:
    // ── Auto-leveling ────────────────────────────────────────

    void updateLeveling(float ax, float ay, float az)
    {
        float a_norm = std::sqrt(ax * ax + ay * ay + az * az);
        if (a_norm <= 6.0f || a_norm >= 14.0f) return;

        acc_accum += Vec3f(ax, ay, az);
        level_samples++;
        if (level_samples >= 50) {
            float norm = acc_accum.norm();
            if (norm > 50.0f * 6.0f) { u_z = acc_accum / norm; is_leveled = true; }
        }
    }

    void updateForwardAxis(float ax, float ay, float az, int8_t sign)
    {
        if (!is_leveled || sign == 0) return;

        Vec3f a(ax, ay, az);
        Vec3f a_horiz = a - a.dot(u_z) * u_z;
        float h_norm = a_horiz.norm();
        if (h_norm < 0.35f || h_norm > 8.0f) return;

        fwd_accum += static_cast<float>(sign) * a_horiz;
        fwd_accum -= fwd_accum.dot(u_z) * u_z;
        fwd_samples++;

        if (fwd_samples >= 15) {
            float fn = fwd_accum.norm();
            if (fn > 3.0f) {
                u_x = fwd_accum / fn;
                u_y = Vec3f(
                    u_z(1) * u_x(2) - u_z(2) * u_x(1),
                    u_z(2) * u_x(0) - u_z(0) * u_x(2),
                    u_z(0) * u_x(1) - u_z(1) * u_x(0)
                ).normalized();
                is_fwd_calibrated = true;
            }
        }
    }

    float projectYawRate(float gx, float gy, float gz) const
    {
        if (!is_leveled) return gz * DEG2RAD_F;
        return (gx * u_z(0) + gy * u_z(1) + gz * u_z(2)) * DEG2RAD_F;
    }

    // ── Prediction (RK2 midpoint) ────────────────────────────

    void predict(float omega_meas, float a_meas, float dt)
    {
        if (dt <= 0.0f) return;
        dt = std::min(dt, 0.5f);
        current_t += dt;

        float x = X(0), y = X(1), th = X(2);
        float v = X(3), bw = X(4), ba = X(5);

        float w     = omega_meas - bw;
        float a     = a_meas - ba;
        float v_eff = std::max(0.0f, v);

        float th_mid = th + 0.5f * w * dt;
        float v_mid  = v_eff + 0.5f * a * dt;
        float cos_m  = std::cos(th_mid);
        float sin_m  = std::sin(th_mid);

        X(0) = x + v_mid * cos_m * dt;
        X(1) = y + v_mid * sin_m * dt;
        X(2) = wrap_angle(th + w * dt);
        X(3) = std::max(0.0f, v + a * dt);

        dist_since_fix   += std::fabs(v_eff) * dt;
        accum_wheel_dist += std::fabs(v_eff) * dt;

        // Jacobian F_mat (7×7)
        Mat7f F_mat = Mat7f::Identity();
        float dt2 = dt * dt;
        F_mat(0,2) = -v_mid * sin_m * dt;  F_mat(0,3) =  cos_m * dt;
        F_mat(0,4) = -0.5f * v_mid * sin_m * dt2;
        F_mat(0,5) = -0.5f * cos_m * dt2;
        F_mat(1,2) =  v_mid * cos_m * dt;  F_mat(1,3) =  sin_m * dt;
        F_mat(1,4) =  0.5f * v_mid * cos_m * dt2;
        F_mat(1,5) = -0.5f * sin_m * dt2;
        F_mat(2,4) = -dt;
        F_mat(3,5) = -dt;

        // Process noise Q
        float q_pos = (0.05f + 0.12f * v_eff); q_pos *= q_pos;
        float q_th  = (0.005f + 0.03f * std::fabs(w)); q_th *= q_th;
        Vec7f qd; qd << q_pos, q_pos, q_th, 0.08f, 1e-8f, 1e-6f, 1e-7f;
        Mat7f Q = qd.asDiagonal();

        P = F_mat * P * F_mat.transpose() + Q * dt;
        ensurePD();
    }

    // ── Wheel-speed correction ───────────────────────────────

    void updateOBD(float v_wheel)
    {
        if (v_wheel < 0.05f) { updateZUPT(); return; }

        float k      = X(6);
        float k_safe = std::max(0.5f, k);
        float y_v    = v_wheel / k_safe - X(3);
        float r_v    = par.r_obd_var / (k * k);
        float s_v    = P(3,3) + r_v;
        if (s_v <= 0.0f || !std::isfinite(s_v)) return;

        Vec7f K = P.col(3) / s_v;
        K(0) = 0; K(1) = 0; K(2) = 0; K(4) = 0; K(6) = 0;

        X += K * y_v;
        X(3) = std::max(0.0f, X(3));

        Mat7f KH = Mat7f::Zero();
        KH.col(3) = K;
        Mat7f I_KH = Mat7f::Identity() - KH;
        P = I_KH * P * I_KH.transpose() + r_v * (K * K.transpose());
        ensurePD();
    }

    // ── ZUPT ─────────────────────────────────────────────────

    void updateZUPT(float w_meas = NAN)
    {
        X(3) = 0.0f;
        P(3,3) = std::min(P(3,3), 0.01f);
        if (std::isfinite(w_meas)) {
            X(4) += 0.05f * (w_meas - X(4));
            P(4,4) = std::min(P(4,4), 1e-5f);
        }
        ensurePD();
    }

    // ── GPS correction ───────────────────────────────────────

    bool updateGPS(float gx, float gy, float gps_acc_m)
    {
        float acc_m = std::max(gps_acc_m, par.min_gps_sigma);
        float dg  = std::hypot(gx - last_fix_x, gy - last_fix_y);
        float dtg = current_t - last_fix_t;

        // Heading init
        if (!heading_init) {
            if (!has_anchor) { anchor_x = gx; anchor_y = gy; has_anchor = true; }
            float d_anc = std::hypot(gx - anchor_x, gy - anchor_y);
            if (d_anc >= 3.0f && dtg > 0.2f) {
                X(2) = std::atan2(gy - anchor_y, gx - anchor_x);
                heading_init = true;
                P(2,2) = 0.03f;
            }
            X(0) = gx; X(1) = gy; prev_x = gx; prev_y = gy;
            last_fix_x = gx; last_fix_y = gy; last_fix_t = current_t;
            dist_since_fix = 0; accum_wheel_dist = 0;
            return true;
        }

        // Innovation
        Vec2f y_inn; y_inn << gx - X(0), gy - X(1);
        float resid = y_inn.norm();
        Mat2f R_gps = Mat2f::Identity() * (acc_m * acc_m);
        Mat2f S     = P.topLeftCorner<2,2>() + R_gps;
        Mat2f S_inv = S.inverse();
        float md2   = (y_inn.transpose() * S_inv * y_inn)(0,0);

        // Gate
        float gate_m   = std::max(15.0f, 3.0f * acc_m + 0.30f * dist_since_fix);
        float v_implied = (dtg > 0.0f) ? dg / dtg : 0.0f;
        bool  spoofed   = (resid > gate_m) || (md2 > par.chi2_gate)
                       || (dtg > 0.0f && v_implied > 60.0f);

        if (spoofed) { is_dr = true; consecutive_good = 0; return false; }

        if (is_dr) {
            if (++consecutive_good < par.recover_n) return false;
            is_dr = false;
        }

        // M-estimation
        if (md2 > 4.6f) {
            R_gps *= md2 / 4.6f;
            S = P.topLeftCorner<2,2>() + R_gps;
            S_inv = S.inverse();
        }

        // Kalman update (position only)
        Eigen::Matrix<float, 7, 2> K = P.leftCols<2>() * S_inv;
        K.bottomRows(5).setZero();
        X += K * y_inn;

        Mat7f KH = Mat7f::Zero();
        KH.leftCols<2>() = K;
        Mat7f I_KH = Mat7f::Identity() - KH;
        P = I_KH * P * I_KH.transpose() + K * R_gps * K.transpose();
        ensurePD();

        // Soft heading correction
        if (dg >= 3.0f && dtg > 0.3f && X(3) > 1.5f) {
            float chord = std::atan2(gy - last_fix_y, gx - last_fix_x);
            X(2) = wrap_angle(X(2) + 0.25f * wrap_angle(chord - X(2)));
            P(2,2) = std::max(0.01f, 0.95f * P(2,2));
        }

        // Online k_slip calibration
        if (dg >= 4.0f && accum_wheel_dist >= 3.0f && dtg > 0.3f) {
            float ratio = accum_wheel_dist / dg;
            float p_kk  = P(6,6);
            float s_r   = p_kk + 0.0025f;
            if (s_r > 0.0f) {
                float k_r = p_kk / s_r;
                X(6) = std::max(0.85f, std::min(1.15f, X(6) + k_r * (ratio - X(6))));
                P(6,6) = std::max(1e-5f, (1.0f - k_r) * p_kk);
            }
            ensurePD();
        }

        last_fix_x = gx; last_fix_y = gy; last_fix_t = current_t;
        dist_since_fix = 0; accum_wheel_dist = 0;
        return true;
    }

    // ── Численная стабильность ────────────────────────────────

    void ensurePD()
    {
        P = 0.5f * (P + P.transpose());
        P(2,2) = std::min(P(2,2), 0.25f);
        P(4,4) = std::min(P(4,4), 1e-4f);
        for (int i = 0; i < 7; i++) P(i,i) = std::max(P(i,i), 1e-7f);
        for (int i = 0; i < 7; i++)
            for (int j = i + 1; j < 7; j++) {
                float bound = std::sqrt(P(i,i) * P(j,j));
                P(i,j) = std::max(-bound, std::min(bound, P(i,j)));
                P(j,i) = P(i,j);
            }
    }
};
