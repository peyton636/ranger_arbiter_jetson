"""Python port of imu_gps_localization C++ EKF core."""
from collections import deque

import numpy as np

from ds_imu_gps_localization.geo_utils import (
    DEG2RAD,
    angle_axis_to_rot,
    enu_to_lla,
    lla_to_enu,
    skew,
)

K_IMU_BUFFER = 100
K_ACC_STD_LIMIT = 3.0


class ImuGpsLocalizer:
    def __init__(
        self,
        acc_noise=1e-2,
        gyro_noise=1e-4,
        acc_bias_noise=1e-6,
        gyro_bias_noise=1e-8,
        i_p_gps=None,
        gravity=None,
    ):
        self.i_p_gps = np.zeros(3) if i_p_gps is None else np.asarray(i_p_gps, float)
        self.gravity = (
            np.array([0.0, 0.0, -9.81007]) if gravity is None else np.asarray(gravity, float)
        )
        self.acc_noise = acc_noise
        self.gyro_noise = gyro_noise
        self.acc_bias_noise = acc_bias_noise
        self.gyro_bias_noise = gyro_bias_noise

        self.initialized = False
        self.init_lla = np.zeros(3)
        self.imu_buffer = deque(maxlen=K_IMU_BUFFER)
        self.state = self._empty_state()

    @staticmethod
    def _empty_state():
        return {
            "timestamp": 0.0,
            "lla": np.zeros(3),
            "G_p_I": np.zeros(3),
            "G_v_I": np.zeros(3),
            "G_R_I": np.eye(3),
            "acc_bias": np.zeros(3),
            "gyro_bias": np.zeros(3),
            "cov": np.eye(15),
            "imu_data": None,
        }

    def add_imu_data(self, imu_data):
        self.imu_buffer.append(imu_data)
        if not self.initialized:
            return None
        if self.state["imu_data"] is None:
            self.state["imu_data"] = imu_data
            return None
        self._predict(self.state["imu_data"], imu_data)
        self.state["lla"] = enu_to_lla(
            self.state["G_p_I"][0],
            self.state["G_p_I"][1],
            self.state["G_p_I"][2],
            self.init_lla[0],
            self.init_lla[1],
            self.init_lla[2],
        )
        return dict(self.state)

    def add_gps_data(self, gps_data):
        if not self.initialized:
            if len(self.imu_buffer) < K_IMU_BUFFER:
                return False
            last_imu = self.imu_buffer[-1]
            if abs(gps_data["timestamp"] - last_imu["timestamp"]) > 0.5:
                return False
            if not self._init_state(last_imu):
                return False
            self.init_lla = gps_data["lla"].copy()
            self.initialized = True
            return True
        self._update_gps(gps_data)
        return True

    def _init_state(self, last_imu):
        g_r_i = self._compute_g_r_i_from_imu()
        if g_r_i is None:
            return False
        self.state["timestamp"] = last_imu["timestamp"]
        self.state["imu_data"] = last_imu
        self.state["G_p_I"] = np.zeros(3)
        self.state["G_v_I"] = np.zeros(3)
        self.state["G_R_I"] = g_r_i
        self.state["acc_bias"] = np.zeros(3)
        self.state["gyro_bias"] = np.zeros(3)
        cov = np.zeros((15, 15))
        cov[0:3, 0:3] = 100.0 * np.eye(3)
        cov[3:6, 3:6] = 100.0 * np.eye(3)
        cov[6:8, 6:8] = (10.0 * DEG2RAD) ** 2 * np.eye(2)
        cov[8, 8] = (100.0 * DEG2RAD) ** 2
        cov[9:12, 9:12] = 0.0004 * np.eye(3)
        cov[12:15, 12:15] = 0.0004 * np.eye(3)
        self.state["cov"] = cov
        return True

    def _compute_g_r_i_from_imu(self):
        accs = np.array([d["acc"] for d in self.imu_buffer])
        mean_acc = accs.mean(axis=0)
        std_acc = accs.std(axis=0)
        if std_acc.max() > K_ACC_STD_LIMIT:
            return None
        z_axis = mean_acc / np.linalg.norm(mean_acc)
        x_axis = np.array([1.0, 0.0, 0.0]) - z_axis * z_axis.dot(np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(x_axis) < 1e-9:
            x_axis = np.array([0.0, 1.0, 0.0]) - z_axis * z_axis.dot(np.array([0.0, 1.0, 0.0]))
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        i_r_g = np.column_stack([x_axis, y_axis, z_axis])
        return i_r_g.T

    def _predict(self, last_imu, cur_imu):
        s = self.state
        last = {
            "G_p_I": s["G_p_I"].copy(),
            "G_v_I": s["G_v_I"].copy(),
            "G_R_I": s["G_R_I"].copy(),
            "acc_bias": s["acc_bias"].copy(),
            "gyro_bias": s["gyro_bias"].copy(),
            "cov": s["cov"].copy(),
        }
        dt = cur_imu["timestamp"] - last_imu["timestamp"]
        dt2 = dt * dt
        acc_unbias = 0.5 * (last_imu["acc"] + cur_imu["acc"]) - last["acc_bias"]
        gyro_unbias = 0.5 * (last_imu["gyro"] + cur_imu["gyro"]) - last["gyro_bias"]

        s["G_p_I"] = (
            last["G_p_I"]
            + last["G_v_I"] * dt
            + 0.5 * (last["G_R_I"] @ acc_unbias + self.gravity) * dt2
        )
        s["G_v_I"] = last["G_v_I"] + (last["G_R_I"] @ acc_unbias + self.gravity) * dt
        delta_angle = gyro_unbias * dt
        s["G_R_I"] = last["G_R_I"] @ angle_axis_to_rot(delta_angle)

        fx = np.eye(15)
        fx[0:3, 3:6] = np.eye(3) * dt
        fx[3:6, 6:9] = -s["G_R_I"] @ skew(acc_unbias) * dt
        fx[3:6, 9:12] = -s["G_R_I"] * dt
        if np.linalg.norm(delta_angle) > 1e-12:
            fx[6:9, 6:9] = angle_axis_to_rot(delta_angle).T
        fx[6:9, 12:15] = -np.eye(3) * dt

        fi = np.zeros((15, 12))
        fi[3:15, 0:12] = np.eye(12)
        qi = np.zeros((12, 12))
        qi[0:3, 0:3] = dt2 * self.acc_noise * np.eye(3)
        qi[3:6, 3:6] = dt2 * self.gyro_noise * np.eye(3)
        qi[6:9, 6:9] = dt * self.acc_bias_noise * np.eye(3)
        qi[9:12, 9:12] = dt * self.gyro_bias_noise * np.eye(3)

        s["cov"] = fx @ last["cov"] @ fx.T + fi @ qi @ fi.T
        s["timestamp"] = cur_imu["timestamp"]
        s["imu_data"] = cur_imu

    def _update_gps(self, gps_data):
        s = self.state
        g_p_gps = lla_to_enu(
            gps_data["lla"][0],
            gps_data["lla"][1],
            gps_data["lla"][2],
            self.init_lla[0],
            self.init_lla[1],
            self.init_lla[2],
        )
        residual = g_p_gps - (s["G_p_I"] + s["G_R_I"] @ self.i_p_gps)
        h = np.zeros((3, 15))
        h[0:3, 0:3] = np.eye(3)
        h[0:3, 6:9] = -s["G_R_I"] @ skew(self.i_p_gps)
        v = gps_data["cov"]
        p = s["cov"]
        k = p @ h.T @ np.linalg.inv(h @ p @ h.T + v)
        delta_x = k @ residual
        self._add_delta(delta_x)
        i_kh = np.eye(15) - k @ h
        s["cov"] = i_kh @ p @ i_kh.T + k @ v @ k.T

    def _add_delta(self, delta_x):
        s = self.state
        s["G_p_I"] += delta_x[0:3]
        s["G_v_I"] += delta_x[3:6]
        s["acc_bias"] += delta_x[9:12]
        s["gyro_bias"] += delta_x[12:15]
        drot = delta_x[6:9]
        if np.linalg.norm(drot) > 1e-12:
            s["G_R_I"] = s["G_R_I"] @ angle_axis_to_rot(drot)
