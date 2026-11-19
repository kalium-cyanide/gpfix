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

#include "config.hpp"
#include "data.hpp"
#include "imodel.hpp"

#include <ArduinoEigen/Eigen/Core>
#include <ArduinoEigen/Eigen/LU>
#include <cmath>

// ── Constants ─────────────────────────────────────────────────
static constexpr float  PI_F       = 3.14159265358979323846f;
static constexpr float  DEG2RAD_F  = PI_F / 180.0f;
static constexpr float  RAD2DEG_F  = 180.0f / PI_F;
static constexpr double R_EARTH_D  = 6378137.0;

// ── Type aliases ──────────────────────────────────────────────
using Vec7f  = Eigen::Matrix<float, 7, 1>;
using Mat7f  = Eigen::Matrix<float, 7, 7>;
using Vec3f  = Eigen::Matrix<float, 3, 1>;
using Vec2f  = Eigen::Matrix<float, 2, 1>;
using Mat2f  = Eigen::Matrix<float, 2, 2>;
using RowVec7f = Eigen::Matrix<float, 1, 7>;

// ── Utilities ─────────────────────────────────────────────────
inline float wrap_angle(float a)
{
    if (!std::isfinite(a)) return NAN;
    a = std::fmod(a + PI_F, 2.0f * PI_F);
    if (a < 0.0f) a += 2.0f * PI_F;
    return a - PI_F;
}

// ── ENU (East-North-Up) ──────────────────────────────────────
struct ENU {
    double lat0 = 0, lon0 = 0;
    double kx = 0;
    bool ready = false;

    void setOrigin(double lat_deg, double lon_deg)
    {
        lat0 = lat_deg;
        lon0 = lon_deg;
        kx   = R_EARTH_D * std::cos(lat_deg * M_PI / 180.0);
        ready = true;
    }

    void fwd(double lat_deg, double lon_deg, float& x_e, float& y_n) const
    {
        x_e = static_cast<float>((lon_deg - lon0) * (M_PI / 180.0) * kx);
        y_n = static_cast<float>((lat_deg - lat0) * (M_PI / 180.0) * R_EARTH_D);
    }

    void inv(float x_e, float y_n, double& lat_deg, double& lon_deg) const
    {
        lat_deg = lat0 + static_cast<double>(y_n) / R_EARTH_D * (180.0 / M_PI);
        lon_deg = lon0 + static_cast<double>(x_e) / kx * (180.0 / M_PI);
    }
};

// ── ESKF params ──────────────────────────────────────────────
struct ESKFParams {
    float wheelbase    = 2.6f;
    float steer_ratio  = 15.0f;
    float c_steer      = 1.0f / (2.6f * 15.0f);
    float chi2_gate    = 9.21f;   // 2-DOF 99 %
    float min_gps_sigma = 2.5f;
    float r_obd_var    = 0.25f;   // (м/с)²
    int   recover_n    = 3;
    float gps_default_acc = 5.0f; // м
};

// ══════════════════════════════════════════════════════════════
//  BicycleESKF : IModel
// ══════════════════════════════════════════════════════════════

struct BicycleESKF : IModel {

    // ── State ────────────────────────────────────────────────
    Vec7f X;
    Mat7f P;
    ESKFParams par;
    ENU enu;

    // ── Tracking ─────────────────────────────────────────────
    float prev_x = 0, prev_y = 0;
    uint64_t last_ts_us = 0;
    bool  heading_init = false;
    bool  is_dr = false;
    int   consecutive_good = 0;
    float dist_since_fix = 0;
    float accum_wheel_dist = 0;
    float last_fix_x = 0, last_fix_y = 0;
    float last_fix_t = 0;
    float current_t = 0;
    bool  has_anchor = false;
    float anchor_x = 0, anchor_y = 0;

    SpeedData latest_speed {};
    uint64_t last_speed_ts = 0;

    // ── 3D Auto-leveling state ───────────────────────────────
    Vec3f u_z = Vec3f(0.0f, 0.0f, 1.0f); // Единичный вектор вертикали (по умолчанию Z вверх)
    bool  is_leveled = false;
    Vec3f acc_accum = Vec3f::Zero();
    uint32_t level_samples = 0;

    void update_leveling(float ax, float ay, float az)
    {
        acc_accum += Vec3f(ax, ay, az);
        level_samples++;
        if (level_samples >= 50) {
            float norm = acc_accum.norm();
            if (norm > 1.0f) {
                u_z = acc_accum / norm;
                is_leveled = true;
            }
        }
    }

    float project_yaw_rate(float gx_dps, float gy_dps, float gz_dps) const
    {
        if (!is_leveled) {
            return gz_dps * DEG2RAD_F;
        }
        // Скалярное произведение 3D угловой скорости на истинную вертикаль
        float w_dps = gx_dps * u_z(0) + gy_dps * u_z(1) + gz_dps * u_z(2);
        return w_dps * DEG2RAD_F;
    }

    // ── Constructor ──────────────────────────────────────────
    explicit BicycleESKF(const ESKFParams& p = {})
        : par(p)
    {
        X.setZero();
        X(6) = 1.0f; // k_slip

        P.setZero();
        P(0,0) = 4.0f;
        P(1,1) = 4.0f;
        P(2,2) = 0.15f;
        P(3,3) = 0.25f;
        P(4,4) = 1e-5f;
        P(5,5) = 0.1f;
        P(6,6) = 0.01f;
    }

    // ══════════════════════════════════════════════════════════
    //  IModel interface
    // ══════════════════════════════════════════════════════════

    DeltaS update(const IMUData& imu, float /*madgwick_yaw*/) override
    {
        // ── Автокалибровка вертикали (в покое или первые 50 отсчётов) ──
        if (!is_leveled || (latest_speed.valid && latest_speed.speed_kmh < 0.2f)) {
            update_leveling(imu.acc_x, imu.acc_y, imu.acc_z);
        }

        // ── dt ───────────────────────────────────────────────
        float dt = 0.0f;
        if (last_ts_us != 0) {
            dt = static_cast<float>(imu.timestamp_us - last_ts_us) * 1e-6f;
            if (dt <= 0.0f || dt > 0.5f) dt = 0.0f;
        }
        last_ts_us = imu.timestamp_us;

        // ── Predict ──────────────────────────────────────────
        float omega = 0.0f;
        if (dt > 0.0f) {
            // Проецируем 3D угловую скорость на истинную вертикаль
            omega = project_yaw_rate(imu.gyro_x, imu.gyro_y, imu.gyro_z);
            // Продольное ускорение: скорость надёжно корректируется колесом (20 Гц)
            predict(omega, 0.0f, dt);
        }

        // ── Wheel speed update (CAN or Hall sensor) ─────────
        if (latest_speed.valid && latest_speed.timestamp_us != last_speed_ts) {
            last_speed_ts = latest_speed.timestamp_us;
            float v_mps = latest_speed.speed_kmh / 3.6f;
            if (v_mps < 0.05f) {
                update_zupt();
            } else {
                update_obd(v_mps);
            }
        }

        // ── Compute delta for BLE ────────────────────────────
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
    }

    void onCAN(const CANData& can) override
    {
        onSpeed(can);
    }

    void onGPS(const GPSData& gps) override
    {
        if (!gps.valid) return;

        // First fix → set ENU origin
        if (!enu.ready) {
            enu.setOrigin(gps.lat, gps.lon);
        }

        float gx, gy;
        enu.fwd(gps.lat, gps.lon, gx, gy);
        update_gps_internal(gx, gy, par.gps_default_acc);
    }

    ~BicycleESKF() override = default;

    // ══════════════════════════════════════════════════════════
    //  Prediction — Bicycle kinematic model  (RK2 midpoint)
    // ══════════════════════════════════════════════════════════

    void predict(float omega_meas, float a_meas, float dt)
    {
        if (dt <= 0.0f) return;
        dt = std::min(dt, 0.5f);
        current_t += dt;

        float x  = X(0), y  = X(1), th = X(2);
        float v  = X(3), bw = X(4), ba = X(5);

        float w = omega_meas - bw;
        float a = a_meas - ba;
        float v_eff = std::max(0.0f, v);

        // RK2 midpoint
        float th_mid = th + 0.5f * w * dt;
        float v_mid  = v_eff + 0.5f * a * dt;
        float cos_m  = std::cos(th_mid);
        float sin_m  = std::sin(th_mid);

        X(0) = x + v_mid * cos_m * dt;
        X(1) = y + v_mid * sin_m * dt;
        X(2) = wrap_angle(th + w * dt);
        X(3) = std::max(0.0f, v + a * dt);

        dist_since_fix     += std::fabs(v_eff) * dt;
        accum_wheel_dist   += std::fabs(v_eff) * dt;

        // ── Jacobian F (7×7) ─────────────────────────────────
        Mat7f F = Mat7f::Identity();
        float dt2 = dt * dt;

        F(0,2) = -v_mid * sin_m * dt;
        F(0,3) =  cos_m * dt;
        F(0,4) = -0.5f * v_mid * sin_m * dt2;
        F(0,5) = -0.5f * cos_m * dt2;

        F(1,2) =  v_mid * cos_m * dt;
        F(1,3) =  sin_m * dt;
        F(1,4) =  0.5f * v_mid * cos_m * dt2;
        F(1,5) = -0.5f * sin_m * dt2;

        F(2,4) = -dt;
        F(3,5) = -dt;

        // ── Dynamic process noise Q ──────────────────────────
        float q_pos   = (0.05f + 0.12f * v_eff);  q_pos *= q_pos;
        float q_th    = (0.005f + 0.03f * std::fabs(w)); q_th *= q_th;
        float q_v     = 0.08f;
        float q_bw    = 1e-8f;
        float q_ba    = 1e-6f;
        float q_kslip = 1e-7f;

        Vec7f q_diag;
        q_diag << q_pos, q_pos, q_th, q_v, q_bw, q_ba, q_kslip;
        Mat7f Q = q_diag.asDiagonal();

        P = F * P * F.transpose() + Q * dt;
        ensure_positive_definite();
    }

    // ══════════════════════════════════════════════════════════
    //  OBD / wheel-speed correction
    // ══════════════════════════════════════════════════════════

    void update_obd(float v_wheel)
    {
        if (v_wheel < 0.05f) {
            update_zupt();
            return;
        }

        float k = X(6);
        float k_safe = std::max(0.5f, k);
        float z_v = v_wheel / k_safe;
        float y_v = z_v - X(3);
        float r_v = par.r_obd_var / (k * k);

        float s_v = P(3,3) + r_v;
        if (s_v <= 0.0f || !std::isfinite(s_v)) return;

        // K = P.col(3) / s_v — only update v (3) and b_a (5)
        Vec7f K = P.col(3) / s_v;
        K(0) = 0; K(1) = 0; K(2) = 0;
        K(4) = 0; K(6) = 0;

        X += K * y_v;
        X(3) = std::max(0.0f, X(3));

        // Joseph form P = (I-K*H) P (I-K*H)^T + K r K^T
        Mat7f KH = Mat7f::Zero();
        KH.col(3) = K;
        Mat7f I_KH = Mat7f::Identity() - KH;
        P = I_KH * P * I_KH.transpose() + (r_v) * (K * K.transpose());
        ensure_positive_definite();
    }

    // ══════════════════════════════════════════════════════════
    //  Zero-velocity update
    // ══════════════════════════════════════════════════════════

    void update_zupt(float w_meas = NAN)
    {
        X(3) = 0.0f;
        P(3,3) = std::min(P(3,3), 0.01f);

        if (std::isfinite(w_meas)) {
            constexpr float alpha = 0.05f;
            X(4) += alpha * (w_meas - X(4));
            P(4,4) = std::min(P(4,4), 1e-5f);
        }
        ensure_positive_definite();
    }

    // ══════════════════════════════════════════════════════════
    //  GPS correction with spoof / jump detection
    // ══════════════════════════════════════════════════════════

    bool update_gps_internal(float gx, float gy, float gps_acc_m)
    {
        float acc_m = std::max(gps_acc_m, par.min_gps_sigma);
        float dg  = std::hypot(gx - last_fix_x, gy - last_fix_y);
        float dtg = current_t - last_fix_t;

        // ── 1. Heading init from first movement ──────────────
        if (!heading_init) {
            if (!has_anchor) {
                anchor_x = gx;  anchor_y = gy;
                has_anchor = true;
            }
            float d_anc = std::hypot(gx - anchor_x, gy - anchor_y);
            if (d_anc >= 3.0f && dtg > 0.2f) {
                X(2) = std::atan2(gy - anchor_y, gx - anchor_x);
                heading_init = true;
                P(2,2) = 0.03f;
            }
            X(0) = gx;  X(1) = gy;
            prev_x = gx; prev_y = gy;
            last_fix_x = gx; last_fix_y = gy;
            last_fix_t = current_t;
            dist_since_fix = 0;  accum_wheel_dist = 0;
            return true;
        }

        // ── 2. Innovation ────────────────────────────────────
        Vec2f y_inn;
        y_inn << gx - X(0), gy - X(1);
        float resid = y_inn.norm();

        Mat2f R_gps = Mat2f::Identity() * (acc_m * acc_m);

        // S = P[0:2,0:2] + R  (since H selects rows 0,1)
        Mat2f S = P.topLeftCorner<2,2>() + R_gps;
        Mat2f S_inv = S.inverse();

        float md2 = (y_inn.transpose() * S_inv * y_inn)(0,0);

        // ── 3. Dynamic gate ──────────────────────────────────
        float gate_m = std::max(15.0f, 3.0f * acc_m + 0.30f * dist_since_fix);
        float v_implied = (dtg > 0.0f) ? dg / dtg : 0.0f;

        bool spoofed = (resid > gate_m) ||
                       (md2 > par.chi2_gate) ||
                       (dtg > 0.0f && v_implied > 60.0f);

        if (spoofed) {
            is_dr = true;
            consecutive_good = 0;
            return false;
        }

        // ── 4. Recovery logic ────────────────────────────────
        if (is_dr) {
            consecutive_good++;
            if (consecutive_good < par.recover_n)
                return false;
            is_dr = false;
        }

        // ── 5. M-estimation ──────────────────────────────────
        if (md2 > 4.6f) {
            float scale = md2 / 4.6f;
            R_gps *= scale;
            S = P.topLeftCorner<2,2>() + R_gps;
            S_inv = S.inverse();
        }

        // ── 6. Kalman update (position only) ─────────────────
        // K = P[:, 0:2] * S_inv   then zero rows 2..6
        Eigen::Matrix<float, 7, 2> K = P.leftCols<2>() * S_inv;
        K.bottomRows(5).setZero();

        X += K * y_inn;

        // Joseph form
        Mat7f KH = Mat7f::Zero();
        KH.leftCols<2>() = K;
        Mat7f I_KH = Mat7f::Identity() - KH;
        P = I_KH * P * I_KH.transpose() + K * R_gps * K.transpose();
        ensure_positive_definite();

        // ── 7. Soft heading correction from chord ────────────
        if (dg >= 3.0f && dtg > 0.3f && X(3) > 1.5f) {
            float chord = std::atan2(gy - last_fix_y, gx - last_fix_x);
            float d_c = wrap_angle(chord - X(2));
            X(2) = wrap_angle(X(2) + 0.25f * d_c);
            P(2,2) = std::max(0.01f, 0.95f * P(2,2));
        }

        // ── 8. Online k_slip calibration ─────────────────────
        if (dg >= 4.0f && accum_wheel_dist >= 3.0f && dtg > 0.3f) {
            float ratio = accum_wheel_dist / dg;
            float p_kk  = P(6,6);
            float r_ratio = 0.0025f; // 0.05²
            float s_ratio = p_kk + r_ratio;
            if (s_ratio > 0.0f) {
                float k_ratio = p_kk / s_ratio;
                float new_k = X(6) + k_ratio * (ratio - X(6));
                X(6) = std::max(0.85f, std::min(1.15f, new_k));
                P(6,6) = std::max(1e-5f, (1.0f - k_ratio) * p_kk);
            }
            ensure_positive_definite();
        }

        last_fix_x = gx;  last_fix_y = gy;
        last_fix_t = current_t;
        dist_since_fix = 0;  accum_wheel_dist = 0;
        return true;
    }

    // ══════════════════════════════════════════════════════════
    //  Numerical stability guard
    // ══════════════════════════════════════════════════════════

    void ensure_positive_definite()
    {
        // Symmetrize
        P = 0.5f * (P + P.transpose());

        // Clamp maximum variances
        P(2,2) = std::min(P(2,2), 0.25f);  // theta
        P(4,4) = std::min(P(4,4), 1e-4f);  // b_w

        // Floor diagonal to prevent collapse
        for (int i = 0; i < 7; i++) {
            P(i,i) = std::max(P(i,i), 1e-7f);
        }

        // Enforce diagonal dominance — clamp off-diag by geometric mean
        for (int i = 0; i < 7; i++) {
            for (int j = i + 1; j < 7; j++) {
                float bound = std::sqrt(P(i,i) * P(j,j));
                P(i,j) = std::max(-bound, std::min(bound, P(i,j)));
                P(j,i) = P(i,j);
            }
        }
    }

    // ── Diagnostics ──────────────────────────────────────────

    float sigma_pos() const
    {
        return std::hypot(std::sqrt(std::max(0.0f, P(0,0))),
                          std::sqrt(std::max(0.0f, P(1,1))));
    }
    float sigma_theta_deg() const
    {
        return std::sqrt(std::max(0.0f, P(2,2))) * RAD2DEG_F;
    }
    bool  isDR() const { return is_dr; }
};
