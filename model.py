"""
model.py — Error-State Kalman Filter (ES-KF) с кинематической/динамической моделью
велосипеда (Bicycle Model) для высокоточной локализации и навигации при сбоях и
искусственном спуфинге GPS.

Основные компоненты:
  1. coord / Coord, ENU: преобразование геодезических координат (WGS-84) в локальную ENU.
  2. IMUFilter: фильтрация шума IMU (ФНЧ) и автоматическое автовыравнивание (leveling)
     3D-акселерометра/гироскопа по вектору гравитации для произвольной ориентации установки сенсора.
  3. VehicleParameters: динамические параметры транспортного средства (масштабный коэффициент
     скорости/slip k_slip, смещения нуля гироскопа/акселерометра, чувствительность руля C_steer).
  4. Model: абстрактный базовый класс для моделей навигации.
  5. BicycleESKF: полнофункциональный Error-State Kalman Filter:
       - Номинальное состояние: [x, y, theta, v, b_w, b_a, k_slip]
       - Ковариация вектора ошибок P (уверенность в параметрах и состояниях)
       - Динамический расчет шума процесса Q (зависит от скорости и маневров)
       - Обновление по OBD / датчику скорости колеса (с онлайн-оценкой k_slip)
       - Zero-velocity updates (ZUPT) на стоянке
       - Обновление по GPS с проверкой расстояния Махаланобиса (Хи-квадрат критерий)
         для надежного отсечения скачков и спуфинга
       - Мягкая коррекция курса по хордам движения
       - Уверенность (1-sigma) доступна для любого параметра модели.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

import numpy as np

R_EARTH = 6_378_137.0


def wrap_angle(a: float) -> float:
    """Нормализация угла в диапазон [-pi, pi]."""
    if isinstance(a, (float, int)) and not math.isfinite(a):
        return float("nan")
    return (a + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class coord:
    """Географические координаты (WGS-84)."""
    lon: float = 0.0
    lat: float = 0.0
    alt: float = 0.0


# Алиас с привычным именованием
Coord = coord

class ENU:
    """Локальная топоцентрическая система координат East-North-Up (ENU)."""

    def __init__(self, lat0: float, lon0: float):
        self.lat0 = float(lat0)
        self.lon0 = float(lon0)
        self.lat0_rad = math.radians(self.lat0)
        self.lon0_rad = math.radians(self.lon0)
        self.kx = R_EARTH * math.cos(self.lat0_rad)

    def fwd(self, lat: float | np.ndarray, lon: float | np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """(lat, lon) в градусах -> (x, y) в метрах."""
        la = np.asarray(lat, dtype=float)
        lo = np.asarray(lon, dtype=float)
        x = np.radians(lo - self.lon0) * self.kx
        y = np.radians(la - self.lat0) * R_EARTH
        return x, y

    def inv(self, x: float | np.ndarray, y: float | np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """(x, y) в метрах -> (lat, lon) в градусах."""
        xa = np.asarray(x, dtype=float)
        ya = np.asarray(y, dtype=float)
        lat = self.lat0 + np.degrees(ya / R_EARTH)
        lon = self.lon0 + np.degrees(xa / self.kx)
        return lat, lon


@dataclass
class VehicleParameters:
    """Динамические и кинематические параметры транспортного средства."""
    wheelbase: float = 2.6          # База L, м (для велосипеда ~1.0, для авто ~2.6-2.9)
    steer_ratio: float = 15.0       # Передаточное отношение руля S_R
    c_steer: float = 1.0 / (2.6 * 15.0) # w = v * C_steer * delta
    k_slip: float = 1.0             # Масштабный коэффициент скорости колеса (v_wheel = k_slip * v)
    b_w: float = 0.0                # Смещение нуля гироскопа по вертикальной оси, рад/с
    b_a: float = 0.0                # Смещение нуля продольного акселерометра, м/с^2
    u_z: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.0])) # Единичный вектор вертикали
    u_x: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0])) # Единичный вектор продольной оси


class IMUFilter:
    """Фильтрация шума сенсоров (ФНЧ) и автовыравнивание 3D IMU по гравитации."""

    def __init__(self, alpha: float = 0.35):
        self.alpha = alpha
        self.g_vec = np.array([0.0, 0.0, -1.0])
        self.u_z = np.array([0.0, 0.0, 1.0])
        self.is_leveled = False
        self.filtered_gyro = np.zeros(3)
        self.filtered_acc = np.zeros(3)
        self.first_sample = True

    def calibrate_stationary_gravity(self, acc_samples: np.ndarray) -> np.ndarray:
        """Оценка вектора гравитации по статическим отсчетам акселерометра."""
        if len(acc_samples) == 0:
            return self.u_z
        mean_acc = np.nanmean(acc_samples, axis=0)
        norm = np.linalg.norm(mean_acc)
        if norm > 1e-3:
            # В покое акселерометр измеряет реакцию опоры: +1g вверх в системе датчика
            self.u_z = -mean_acc / norm
            self.g_vec = mean_acc
            self.is_leveled = True
        return self.u_z

    def filter_sample(self, gyro_xyz: np.ndarray, acc_xyz: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """ФНЧ фильтрация отсчетов IMU."""
        g = np.asarray(gyro_xyz, dtype=float)
        a = np.asarray(acc_xyz, dtype=float) if acc_xyz is not None else np.zeros(3)
        if self.first_sample:
            self.filtered_gyro = g.copy()
            self.filtered_acc = a.copy()
            self.first_sample = False
        else:
            self.filtered_gyro = (1.0 - self.alpha) * self.filtered_gyro + self.alpha * g
            self.filtered_acc = (1.0 - self.alpha) * self.filtered_acc + self.alpha * a
        return self.filtered_gyro, self.filtered_acc

    def project_yaw_rate(self, gyro_xyz: np.ndarray) -> float:
        """Проекция 3D вектора угловой скорости на вертикальную ось рыскания."""
        if len(gyro_xyz) == 1:
            return float(gyro_xyz[0])
        return float(np.dot(gyro_xyz, self.u_z))


class Model(ABC):
    """Абстрактный базовый класс для навигационной модели."""

    @abstractmethod
    def step(self, dt: float, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """Выполняет один шаг фильтра/модели по поступившим данным сенсоров."""
        pass

    @abstractmethod
    def predict(self, *args, **kwargs) -> None:
        """Шаг предсказания по кинематической модели."""
        pass

    @abstractmethod
    def update_gps(self, *args, **kwargs) -> Tuple[bool, float, str]:
        """Шаг коррекции по GPS с проверкой спуфинга."""
        pass

    def update_mag(self, *args, **kwargs) -> Tuple[bool, float, str]:
        """Шаг коррекции по магнитометру."""
        return False, 0.0, "not_implemented"

    @abstractmethod
    def get_state(self) -> Dict[str, Any]:
        """Возвращает текущую оценку состояния."""
        pass


class BicycleESKF(Model):
    """
    Error-State Kalman Filter (ES-KF) на основе кинематической модели велосипеда
    для слияния данных IMU + OBD/Hall + GPS с детекцией спуфинга и автокалибровкой.

    Вектор номинального состояния X (7 состояний):
      X[0] = x      : Восточное смещение (East), м
      X[1] = y      : Северное смещение (North), м
      X[2] = theta  : Курсовой угол (yaw), рад
      X[3] = v      : Продольная скорость, м/с
      X[4] = b_w    : Смещение нуля гироскопа рыскания, рад/с
      X[5] = b_a    : Смещение нуля продольного акселерометра, м/с^2
      X[6] = k_slip : Коэффициент масштаба скорости колеса / slip ratio

    Ковариационная матрица вектора ошибок P (7x7) хранит дисперсию и взаимную
    уверенность фильтра в каждом параметре.
    """

    def __init__(
        self,
        init_x: float = 0.0,
        init_y: float = 0.0,
        init_theta: Optional[float] = 0.0,
        init_v: float = 0.0,
        init_b_w: float = 0.0,
        init_k_slip: float = 1.0,
        wheelbase: float = 2.6,
        steer_ratio: float = 15.0,
        c_steer: Optional[float] = None,
        chi2_gate: float = 9.21,      # Порог Хи-квадрат для 2 DOF (99%)
        min_gps_sigma: float = 2.5,   # Минимальное СКО шума GPS, м
        r_obd_var: float = 0.25,      # Дисперсия шума OBD, (м/с)^2
        recover_n: int = 3,           # Число согласованных фиксов для выхода из DR
        heading_initialized: Optional[bool] = None,
    ):
        # Номинальное состояние: [x, y, theta, v, b_w, b_a, k_slip]
        th_val = 0.0 if init_theta is None else float(init_theta)
        self.X = np.array([
            float(init_x),
            float(init_y),
            th_val,
            float(init_v),
            float(init_b_w),
            0.0,
            float(init_k_slip)
        ], dtype=float)

        # Ковариация P вектора ошибок (уверенность в параметрах)
        # Диагональ: [pos_x, pos_y, theta, v, b_w, b_a, k_slip]
        self.P = np.diag([
            4.0,       # pos_x: (2м)^2
            4.0,       # pos_y: (2м)^2
            0.15,      # theta: ~22 deg
            0.25,      # v: (0.5 м/с)^2
            1e-5,      # b_w: точность калибровки нуля гироскопа
            0.1,       # b_a: точность нуля акселерометра
            0.01       # k_slip: уверенность в коэффициенте колеса
        ])

        # Шум измерений
        self.R_obd = float(r_obd_var)
        self.min_gps_sigma = float(min_gps_sigma)
        self.chi2_gate = float(chi2_gate)
        self.recover_n = int(recover_n)

        # Параметры велосипедной модели
        cs = float(c_steer) if c_steer is not None else 1.0 / (wheelbase * steer_ratio)
        self.params = VehicleParameters(
            wheelbase=wheelbase,
            steer_ratio=steer_ratio,
            c_steer=cs,
            k_slip=init_k_slip,
            b_w=init_b_w
        )

        # Флаги состояния и трекинг разрывов/спуфинга
        if heading_initialized is not None:
            self.heading_initialized = bool(heading_initialized)
        elif init_theta is not None:
            self.heading_initialized = True
        else:
            self.heading_initialized = False

        self.anchor_fix: Optional[Tuple[float, float]] = None
        self.is_dr = False                 # Режим Dead Reckoning (при обрыве или спуфинге)
        self.consecutive_good = 0          # Счетчик подряд идущих согласованных фиксов
        self.dist_since_fix = 0.0          # Пройденный путь после последнего принятого фикса
        self.last_accepted_fix = (float(init_x), float(init_y))
        self.last_fix_t = 0.0
        self.current_t = 0.0

        # Статистика и логирование
        self.events: list = []             # (t, status, resid, gate, md2)
        self.imu_filter = IMUFilter(alpha=0.35)

        self.accumulated_wheel_dist = 0.0  # Интегрированное расстояние по колесам между фиксами GPS
        self.mag_anomaly_streak = 0
        self.mag_recover_n = 8

    def _ensure_positive_definite(self) -> None:
        """Гарантирует симметричность и строгую положительную определенность ковариации P."""
        self.P = 0.5 * (self.P + self.P.T)
        self.P[2, 2] = min(float(self.P[2, 2]), 0.25)
        self.P[4, 4] = min(float(self.P[4, 4]), 1e-4)
        eigvals, eigvecs = np.linalg.eigh(self.P)
        if np.any(eigvals < 1e-7):
            eigvals = np.maximum(eigvals, 1e-7)
            self.P = eigvecs @ np.diag(eigvals) @ eigvecs.T
            self.P = 0.5 * (self.P + self.P.T)

    # ----------------------------------------------------------------------- #
    #  Шаг предсказания (Prediction)
    # ----------------------------------------------------------------------- #
    def predict(
        self,
        omega_meas: float,
        a_meas: float = 0.0,
        dt: float = 0.05,
        steer_angle_rad: Optional[float] = None
    ) -> None:
        """
        Интегрирование кинематической модели велосипеда (номинальное состояние)
        и распространение ковариации ошибок P = F P F.T + Q * dt.
        """
        if dt <= 0.0:
            return
        dt = float(min(dt, 0.5))
        self.current_t += dt

        x, y, th, v, bw, ba, k_slip = self.X

        # Расчет эффективной угловой скорости
        w_gyro = omega_meas - bw
        if steer_angle_rad is not None and not math.isnan(steer_angle_rad):
            w_model = v * self.params.c_steer * steer_angle_rad
            w = 0.85 * w_gyro + 0.15 * w_model
        else:
            w = w_gyro

        a = a_meas - ba
        v_eff = max(0.0, v)

        # Интегрирование методом средней точки (RK2 midpoint)
        th_mid = th + 0.5 * w * dt
        v_mid = v_eff + 0.5 * a * dt

        x_new = x + v_mid * math.cos(th_mid) * dt
        y_new = y + v_mid * math.sin(th_mid) * dt
        th_new = wrap_angle(th + w * dt)
        v_new = max(0.0, v + a * dt)

        self.X[0] = x_new
        self.X[1] = y_new
        self.X[2] = th_new
        self.X[3] = v_new
        self.dist_since_fix += abs(v_eff) * dt
        self.accumulated_wheel_dist += abs(v_eff) * dt

        # Матрица Якобиана ошибок F (7x7)
        F = np.eye(7)
        F[0, 2] = -v_mid * math.sin(th_mid) * dt
        F[0, 3] = math.cos(th_mid) * dt
        F[0, 4] = -0.5 * v_mid * math.sin(th_mid) * (dt ** 2)
        F[0, 5] = -0.5 * math.cos(th_mid) * (dt ** 2)

        F[1, 2] = v_mid * math.cos(th_mid) * dt
        F[1, 3] = math.sin(th_mid) * dt
        F[1, 4] = 0.5 * v_mid * math.cos(th_mid) * (dt ** 2)
        F[1, 5] = -0.5 * math.sin(th_mid) * (dt ** 2)

        F[2, 4] = -dt
        F[3, 5] = -dt

        # Динамический шум процесса Q
        q_pos = (0.05 + 0.12 * v_eff) ** 2
        q_th = (0.005 + 0.03 * abs(w)) ** 2
        q_v = 0.08
        q_bw = 1e-8
        q_ba = 1e-6
        q_kslip = 1e-7

        Q = np.diag([q_pos, q_pos, q_th, q_v, q_bw, q_ba, q_kslip])

        self.P = F @ self.P @ F.T + Q * dt
        self._ensure_positive_definite()

    # ----------------------------------------------------------------------- #
    #  Шаг коррекции по скорости колес / OBD (Update OBD)
    # ----------------------------------------------------------------------- #
    def update_obd(self, v_wheel: float) -> None:
        """
        Коррекция скорости по данным OBD/Hall:
          v = v_wheel / k_slip.
        """
        v_w = float(v_wheel)
        if v_w < 0.05:
            self.update_zupt()
            return

        k_slip = self.X[6]
        # Непосредственное измерение скорости: z = v_wheel / k_slip
        z_v = v_w / max(0.5, k_slip)
        y_v = z_v - self.X[3]
        r_v = self.R_obd / (k_slip ** 2)

        p_vv = self.P[3, 3]
        s_v = p_vv + r_v
        if s_v > 0.0 and math.isfinite(s_v):
            H = np.zeros((1, 7), dtype=float)
            H[0, 3] = 1.0

            K = self.P @ H.T / s_v  # (7, 1)
            # Обновляем скорость и bias акселерометра; координаты и курс изолированы
            K[0:3, 0] = 0.0
            K[4, 0] = 0.0
            K[6, 0] = 0.0

            dx = (K * y_v).flatten()
            self.X += dx
            self.X[3] = max(0.0, self.X[3])

            I_KH = np.eye(7) - K @ H
            self.P = I_KH @ self.P @ I_KH.T + (K * r_v) @ K.T
            self._ensure_positive_definite()

    # ----------------------------------------------------------------------- #
    #  Zero-Velocity Update (ZUPT)
    # ----------------------------------------------------------------------- #
    def update_zupt(self, w_meas_stationary: Optional[float] = None) -> None:
        """Коррекция в неподвижном состоянии: скорость 0, уточнение bias гироскопа."""
        self.X[3] = 0.0
        self.P[3, 3] = min(self.P[3, 3], 0.01)

        if w_meas_stationary is not None and math.isfinite(w_meas_stationary):
            a_b = 0.05
            self.X[4] += a_b * (w_meas_stationary - self.X[4])
            self.params.b_w = self.X[4]
            self.P[4, 4] = min(self.P[4, 4], 1e-5)
        self._ensure_positive_definite()

    # ----------------------------------------------------------------------- #
    #  Коррекция по магнитометру (Update Magnetometer / Compass)
    # ----------------------------------------------------------------------- #
    def update_mag(
        self,
        mag_data: float | Tuple[float, ...] | np.ndarray,
        r_mag_var: float = 0.003,
        declination_deg: float = 12.4,
        chi2_gate: float = 9.0
    ) -> Tuple[bool, float, str]:
        """
        Коррекция ориентации (курса theta и смещения гироскопа b_w) по магнитометру.

        Параметры:
          mag_data: абсолютный угол курса в радианах (или градусах если abs > 2*pi),
                    либо 3D вектор магнитного поля [Bx, By, Bz] в мкТл.
          r_mag_var: дисперсия шума магнитометра (рад^2), по умолчанию ~ (3°)^2 = 0.003.
          declination_deg: магнитное склонение региона (по умолчанию +12.4° для Москвы).
          chi2_gate: порог Хи-квадрат (1 DOF, 9.0 ~ 3-sigma) для отсечения магнитных аномалий.

        Возвращает:
          (accepted: bool, mahalanobis_d2: float, status: str)
        """
        if isinstance(mag_data, (list, tuple, np.ndarray)) and len(mag_data) >= 3:
            b = np.asarray(mag_data[:3], dtype=float)
            u_z = self.imu_filter.u_z
            v_ref = np.array([0.0, 1.0, 0.0])
            u_y = v_ref - np.dot(v_ref, u_z) * u_z
            ny = np.linalg.norm(u_y)
            u_y = u_y / ny if ny > 1e-4 else np.array([0.0, 1.0, 0.0])
            u_x = np.cross(u_y, u_z)
            nx = np.linalg.norm(u_x)
            u_x = u_x / nx if nx > 1e-4 else np.array([1.0, 0.0, 0.0])
            b_fwd = float(np.dot(b, u_x))
            b_left = float(np.dot(b, u_y))
            decl_rad = math.radians(declination_deg)
            z_mag = wrap_angle(math.atan2(b_fwd, b_left) - decl_rad)
        else:
            z_val = float(mag_data)
            if not math.isfinite(z_val):
                return False, 0.0, "nan"
            if abs(z_val) > 2.0 * math.pi:
                z_mag = wrap_angle(math.radians(z_val))
            else:
                z_mag = wrap_angle(z_val)

        # 1. Если курс еще не инициализирован (например, при стоянке до первого GPS)
        if not self.heading_initialized:
            self.X[2] = z_mag
            self.heading_initialized = True
            self.P[2, 2] = min(self.P[2, 2], r_mag_var)
            return True, 0.0, "init"

        # 2. Невязка инновации курса с учетом периодичности [-pi, pi]
        y_inn = wrap_angle(z_mag - self.X[2])

        # 3. Дисперсия инновации: S = P_22 + R_mag
        s_inn = self.P[2, 2] + r_mag_var
        if s_inn <= 0.0 or not math.isfinite(s_inn):
            return False, 0.0, "degenerate_s"

        # 4. Проверка на локальную магнитную аномалию (Chi-square gating 1 DOF)
        d2 = (y_inn ** 2) / s_inn
        if d2 > chi2_gate:
            self.mag_anomaly_streak += 1
            # Если расхождение стабильно держится более mag_recover_n отсчетов подряд
            # — принимаем, что произошла переориентация машины, а не кратковременный люк/рельс
            if self.mag_anomaly_streak >= self.mag_recover_n:
                self.mag_anomaly_streak = 0
                self.P[2, 2] = max(self.P[2, 2], 0.25)
                s_inn = self.P[2, 2] + r_mag_var
                d2 = (y_inn ** 2) / s_inn
            else:
                return False, float(d2), "anomaly"
        else:
            self.mag_anomaly_streak = 0

        # 5. Коэффициент Калмана K (обновляем только курс theta и bias гироскопа b_w)
        H = np.zeros((1, 7), dtype=float)
        H[0, 2] = 1.0
        K = self.P @ H.T / s_inn  # (7, 1)

        K[0:2, 0] = 0.0
        K[3, 0] = 0.0
        K[5:7, 0] = 0.0

        # Ограничение шага коррекции bias гироскопа от магнитометра
        if abs(K[4, 0] * y_inn) > 0.01:
            K[4, 0] = np.sign(K[4, 0] * y_inn) * 0.01 / (y_inn + 1e-9)

        # 6. Обновление вектора состояния
        dx = (K * y_inn).flatten()
        self.X += dx
        self.X[2] = wrap_angle(self.X[2])
        self.params.b_w = self.X[4]

        # 7. Обновление ковариации по форме Джозефа
        I_KH = np.eye(7) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + (K * r_mag_var) @ K.T
        self._ensure_positive_definite()

        return True, float(d2), "accepted"

    # ----------------------------------------------------------------------- #
    #  Шаг коррекции по GPS (Update GPS с обнаружением спуфинга)
    # ----------------------------------------------------------------------- #
    def update_gps(
        self,
        gps_x: float,
        gps_y: float,
        gps_acc_m: float = 5.0,
        t_now: Optional[float] = None
    ) -> Tuple[bool, float, str]:
        """
        Коррекция по координатам GPS с фильтрацией спуфинга и скачков.

        Возвращает:
          (accepted: bool, mahalanobis_d2: float, status: str)
          status: 'init' | 'accepted' | 'rejected' | 'pending' | 'recovered'
        """
        t = self.current_t if t_now is None else float(t_now)
        gx, gy = float(gps_x), float(gps_y)
        acc_m = max(float(gps_acc_m), self.min_gps_sigma)

        dg = math.hypot(gx - self.last_accepted_fix[0], gy - self.last_accepted_fix[1])
        dtg = t - self.last_fix_t

        # 1. Захват начального курса по первому достоверному отрезку движения
        if not self.heading_initialized:
            if self.anchor_fix is None:
                self.anchor_fix = (gx, gy)
            d_anchor = math.hypot(gx - self.anchor_fix[0], gy - self.anchor_fix[1])
            if d_anchor >= 3.0 and dtg > 0.2:
                course = math.atan2(gy - self.anchor_fix[1], gx - self.anchor_fix[0])
                self.X[2] = course
                self.heading_initialized = True
                self.P[2, 2] = 0.03
            self.X[0], self.X[1] = gx, gy
            self.last_accepted_fix = (gx, gy)
            self.last_fix_t = t
            self.dist_since_fix = 0.0
            self.accumulated_wheel_dist = 0.0
            self.events.append((t, "init", 0.0, 0.0, 0.0))
            return True, 0.0, "init"

        # 2. Невязка и ковариация инновации
        H = np.zeros((2, 7), dtype=float)
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        y_inn = np.array([gx - self.X[0], gy - self.X[1]], dtype=float)
        resid = float(np.linalg.norm(y_inn))

        R_gps = np.eye(2) * (acc_m ** 2)
        S = H @ self.P @ H.T + R_gps
        try:
            inv_S = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            inv_S = np.eye(2) / (acc_m ** 2)

        md2 = float(y_inn.T @ inv_S @ y_inn)

        # 3. Динамический допуск (gate): растет с накоплением дрейфа модели (hsig ~ 0.25 рад)
        drift_frac = 0.30
        gate_m = max(15.0, 3.0 * acc_m + drift_frac * self.dist_since_fix)

        # Физическая правдоподобность скорости
        v_gps_implied = dg / dtg if dtg > 0 else 0.0
        v_max_plausible = 60.0  # м/с (~216 км/ч)

        # Критерий отбраковки спуфинга / аномальных скачков
        # Скачок отбрасывается, если нарушен физический допуск gate_m, либо критерий Хи-квадрат md2,
        # либо неправдоподобна скорость между отсчетами GPS.
        is_spoofed = (resid > gate_m) or (md2 > self.chi2_gate) or (dtg > 0 and v_gps_implied > v_max_plausible)

        if is_spoofed:
            self.is_dr = True
            self.consecutive_good = 0
            self.events.append((t, "rejected", resid, gate_m, md2))
            return False, md2, "rejected"

        # 4. Логика восстановления после спуфинга/обрыва (N согласованных фиксов)
        if self.is_dr:
            self.consecutive_good += 1
            if self.consecutive_good < self.recover_n:
                self.events.append((t, "pending", resid, gate_m, md2))
                return False, md2, "pending"
            # Восстановление подтверждено
            self.is_dr = False
            status = "recovered"
        else:
            status = "accepted"

        # 5. Мягкое адаптивное взвешивание при умеренных невязках (M-estimation)
        if md2 > 4.6:
            scale = md2 / 4.6
            R_gps *= scale
            S = H @ self.P @ H.T + R_gps
            try:
                inv_S = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                inv_S = np.eye(2) / (acc_m ** 2)

        # 6. Применение коррекции Калмана через форму Джозефа
        K = self.P @ H.T @ inv_S # (7, 2)
        # Позиционный GPS обновляет только координаты X и Y; курс и параметры не искажаются
        K[2:, :] = 0.0

        dx = (K @ y_inn).flatten()
        self.X += dx

        I_KH = np.eye(7) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_gps @ K.T
        self._ensure_positive_definite()

        # 7. Мягкая коррекция курса по хорде движения при стабильной скорости
        if dg >= 3.0 and dtg > 0.3 and self.X[3] > 1.5:
            chord_course = math.atan2(gy - self.last_accepted_fix[1], gx - self.last_accepted_fix[0])
            d_course = wrap_angle(chord_course - self.X[2])
            gain_course = 0.25
            self.X[2] = wrap_angle(self.X[2] + gain_course * d_course)
            self.P[2, 2] = max(0.01, 0.95 * self.P[2, 2])

        # 8. Онлайн-калибровка k_slip по сравнению пройденного пути GPS и колес
        if dg >= 4.0 and self.accumulated_wheel_dist >= 3.0 and dtg > 0.3:
            meas_ratio = self.accumulated_wheel_dist / dg
            # Фильтр Калмана для одного параметра k_slip
            p_kk = self.P[6, 6]
            r_ratio = 0.05 ** 2
            s_ratio = p_kk + r_ratio
            if s_ratio > 0.0:
                k_ratio = p_kk / s_ratio
                self.X[6] = float(np.clip(self.X[6] + k_ratio * (meas_ratio - self.X[6]), 0.85, 1.15))
                self.P[6, 6] = max(1e-5, (1.0 - k_ratio) * p_kk)
                self.params.k_slip = self.X[6]
            self._ensure_positive_definite()

        self.last_accepted_fix = (gx, gy)
        self.last_fix_t = t
        self.dist_since_fix = 0.0
        self.accumulated_wheel_dist = 0.0
        self.events.append((t, status, resid, gate_m, md2))
        return True, md2, status

    # ----------------------------------------------------------------------- #
    #  Унифицированный интерфейс Model.step
    # ----------------------------------------------------------------------- #
    def step(self, dt: float, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Один шаг обработки поступившего пакета данных сенсоров.
        sensor_data может содержать:
          'gyro': float или list[float]
          'acc': float или list[float] (опц.)
          'speed': float (OBD / колесо в м/с)
          'steer': float (угол руля в рад, опц.)
          'gps': tuple(x, y, acc_m) (опц., если есть новый фикс)
        """
        gyro = sensor_data.get("gyro", 0.0)
        if isinstance(gyro, (list, tuple, np.ndarray)):
            w = self.imu_filter.project_yaw_rate(np.asarray(gyro))
        else:
            w = float(gyro)

        acc = sensor_data.get("acc", 0.0)
        a = float(acc[0]) if isinstance(acc, (list, tuple, np.ndarray)) else float(acc)
        steer = sensor_data.get("steer", None)

        # Предсказание
        self.predict(w, a, dt, steer_angle_rad=steer)

        # Коррекция по скорости OBD
        if "speed" in sensor_data and sensor_data["speed"] is not None:
            self.update_obd(float(sensor_data["speed"]))

        # Коррекция по магнитометру
        mag_result = None
        if "mag" in sensor_data and sensor_data["mag"] is not None:
            mag_result = self.update_mag(sensor_data["mag"])

        # Коррекция по GPS
        gps_result = None
        if "gps" in sensor_data and sensor_data["gps"] is not None:
            gx, gy = sensor_data["gps"][0], sensor_data["gps"][1]
            g_acc = sensor_data["gps"][2] if len(sensor_data["gps"]) > 2 else 5.0
            gps_result = self.update_gps(gx, gy, g_acc)

        return {
            "state": self.get_state(),
            "confidence": self.get_confidence(),
            "gps_result": gps_result,
            "mag_result": mag_result,
            "is_dr": self.is_dr
        }

    # ----------------------------------------------------------------------- #
    #  Получение состояния и диагностика
    # ----------------------------------------------------------------------- #
    def get_state(self) -> Dict[str, Any]:
        """Возвращает текущие оценки состояния фильтра."""
        return {
            "x": float(self.X[0]),
            "y": float(self.X[1]),
            "theta": float(self.X[2]),
            "theta_deg": float(np.degrees(self.X[2])),
            "v": float(self.X[3]),
            "v_kmh": float(self.X[3] * 3.6),
            "b_w": float(self.X[4]),
            "b_w_deg_s": float(np.degrees(self.X[4])),
            "b_a": float(self.X[5]),
            "k_slip": float(self.X[6]),
            "is_dr": bool(self.is_dr)
        }

    def get_confidence(self) -> Dict[str, float]:
        """
        Возвращает параметры уверенности (1-sigma СКО) по всем компонентам
        из ковариационной матрицы P.
        """
        diag = np.maximum(0.0, np.diag(self.P))
        sigmas = np.sqrt(diag)
        return {
            "sigma_pos_x_m": float(sigmas[0]),
            "sigma_pos_y_m": float(sigmas[1]),
            "sigma_pos_total_m": float(math.hypot(sigmas[0], sigmas[1])),
            "sigma_theta_deg": float(np.degrees(sigmas[2])),
            "sigma_v_m_s": float(sigmas[3]),
            "sigma_bw_deg_s": float(np.degrees(sigmas[4])),
            "sigma_ba_m_s2": float(sigmas[5]),
            "sigma_k_slip": float(sigmas[6])
        }
