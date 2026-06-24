"""BLOB 上行缓存 → 对外 VehicleData / AgvFeatureStatus。"""

from __future__ import annotations

import time

from jetson_mcu_msgs.msg import V3Command, V3ExtStatus, V3Status
from scr_sensor.msg import AgvFeatureStatus, AgvMotorCompact, VehicleData

from eth_gateway.blob_codec import BlobUplinkCache, cache_to_v3_ext, cache_to_v3_status

SONAR_SRC_MCU = 0
SONAR_SRC_SENSOR = 1


def _motor_msg(m) -> AgvMotorCompact:
    out = AgvMotorCompact()
    out.speed_rpm = int(m.speed_rpm)
    out.current = int(m.current)
    out.voltage = int(m.voltage)
    out.temperature = int(m.temperature)
    out.driver_status = int(m.driver_status)
    out.position_lo = int(m.position_lo)
    return out


def _decode_light_pack(pack: int) -> tuple[int, int]:
    return pack & 0x01, (pack >> 1) & 0x01


def build_vehicle_data(
    cache: BlobUplinkCache,
    status: V3Status | None,
    ext: V3ExtStatus | None,
    *,
    last_status_time: float,
    status_timeout_s: float,
) -> VehicleData | None:
    if status is None:
        stamp_status = cache_to_v3_status(cache, None)
        if stamp_status is None:
            return None
        status = stamp_status
    msg = VehicleData()
    msg.header = status.header
    msg.linear_velocity_mm_s = status.fb_v_mm_s
    msg.angular_velocity_millirad_s = status.fb_omega_millirad_s
    msg.steer_millirad = status.fb_steer_millirad
    msg.sonar_front_mm = status.sonar_front_mm
    msg.sonar_back_mm = status.sonar_back_mm
    msg.sonar_left_mm = status.sonar_left_mm
    msg.sonar_right_mm = status.sonar_right_mm
    msg.battery_voltage_0p1v = status.battery_voltage_0p1v
    msg.battery_soc_percent = status.battery_soc
    msg.v3_status_seq = status.seq
    msg.data_valid = (time.monotonic() - last_status_time) <= status_timeout_s

    if cache.sensor is not None:
        msg.sonar_source = SONAR_SRC_SENSOR
    elif cache.mcu is not None:
        msg.sonar_source = SONAR_SRC_MCU

    if ext is None:
        ext = cache_to_v3_ext(cache, status.header.stamp)
    if ext is not None:
        msg.wheel_rf = ext.wheel_rf
        msg.wheel_rr = ext.wheel_rr
        msg.wheel_lr = ext.wheel_lr
        msg.wheel_lf = ext.wheel_lf
        msg.steer_rf_millirad = ext.steer_rf_millirad
        msg.steer_rr_millirad = ext.steer_rr_millirad
        msg.steer_lr_millirad = ext.steer_lr_millirad
        msg.steer_lf_millirad = ext.steer_lf_millirad
        msg.motor_temp_max_c = ext.motor_temp_max_c
        msg.driver_state_or = ext.driver_state_or
        msg.v3_ext_status_seq = ext.seq

    if cache.energy is not None:
        e = cache.energy
        msg.bms_soh_percent = e.bms_soh
        msg.bms_current = e.bms_current
        msg.bms_temperature = e.bms_temperature
        msg.bms_alarm1 = e.bms_alarm1
        msg.bms_alarm2 = e.bms_alarm2
        msg.bms_warning1 = e.bms_warning1
        msg.bms_warning2 = e.bms_warning2
        msg.odom_pulse = [int(x) for x in e.odom]
        if e.bms_voltage_0p1v:
            msg.battery_voltage_0p1v = e.bms_voltage_0p1v
        if e.bms_soc:
            msg.battery_soc_percent = e.bms_soc

    if cache.motors:
        motors = [_motor_msg(m) for m in cache.motors]
        while len(motors) < 8:
            motors.append(AgvMotorCompact())
        msg.motors = motors

    if cache.motor_pos is not None:
        msg.motor_position = [int(x) for x in cache.motor_pos.motor_position]

    return msg


def build_feature_status(
    cache: BlobUplinkCache,
    status: V3Status | None,
    *,
    jetson_link_up: bool,
    last_status_time: float,
    status_timeout_s: float,
    time_sync_valid: bool,
    time_sync_offset_ms: float,
) -> AgvFeatureStatus | None:
    if status is None:
        status = cache_to_v3_status(cache, None)
        if status is None:
            return None

    msg = AgvFeatureStatus()
    msg.header = status.header
    msg.safety_state = status.safety_state
    msg.link_state = status.link_state
    msg.limit_factor = status.limit_factor
    msg.v3_status_seq = status.seq
    msg.data_valid = (time.monotonic() - last_status_time) <= status_timeout_s
    msg.jetson_link_up = jetson_link_up
    msg.time_sync_valid = time_sync_valid
    msg.time_sync_offset_ms = time_sync_offset_ms

    if cache.mcu is not None:
        msg.nearest_obstacle_mm = cache.mcu.nearest_mm
        msg.jetson_seq = cache.mcu.jetson_seq
        msg.arb_v_mm_s = cache.mcu.arb_v
        msg.arb_omega_millirad_s = cache.mcu.arb_w
        msg.arb_steer_millirad = cache.mcu.arb_steer

    if cache.motion is not None:
        m = cache.motion
        msg.motion_model = m.motion_info
        msg.fault_code = m.fault_code
        msg.system_info = m.system_info
        en, mode = _decode_light_pack(m.light_pack)
        msg.light_enable_feedback = en
        msg.light_mode_feedback = mode
    else:
        msg.fault_code = 0

    return msg


def infer_motion_model(v_mm_s: int, omega: int, steer: int) -> int:
    if abs(steer) >= 800:
        return V3Command.MOTION_SIDEWAYS
    if abs(omega) >= 25 and abs(v_mm_s) < 15:
        return V3Command.MOTION_SPIN
    if abs(v_mm_s) < 15 and abs(omega) < 25:
        return V3Command.MOTION_PARK
    return V3Command.MOTION_ACKERMANN
