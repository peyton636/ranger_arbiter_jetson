"""V3 帧 ↔ jetson_can_msgs 转换。"""

from __future__ import annotations

from jetson_can_msgs.msg import V3Command, V3ExtStatus, V3Status

from ds_jetson_bridge.jetson_protocol import (
    MODE_CAN,
    DownlinkCommand,
    UplinkExt,
    UplinkStatus,
    encode_downlink,
    encode_stop_downlink,
)

SONAR_INVALID = 65535


def _sonar_to_u16(value: int | None) -> int:
    if value is None:
        return SONAR_INVALID
    return value


def uplink_status_to_msg(status: UplinkStatus, stamp) -> V3Status:
    msg = V3Status()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.seq = status.seq
    msg.safety_state = status.safety_state
    msg.link_state = status.link_state
    msg.limit_factor = status.limit_factor
    msg.fb_v_mm_s = status.v_actual_mm_s
    msg.fb_omega_millirad_s = status.omega_millirad_s
    msg.fb_steer_millirad = status.steer_millirad
    msg.sonar_front_mm = _sonar_to_u16(status.sonar_front_mm)
    msg.sonar_back_mm = _sonar_to_u16(status.sonar_back_mm)
    msg.sonar_left_mm = _sonar_to_u16(status.sonar_left_mm)
    msg.sonar_right_mm = _sonar_to_u16(status.sonar_right_mm)
    msg.battery_voltage_0p1v = status.battery_voltage_0p1v
    msg.battery_soc = status.battery_soc
    return msg


def uplink_ext_to_msg(ext: UplinkExt, stamp) -> V3ExtStatus:
    msg = V3ExtStatus()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.seq = ext.seq
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
    return msg


def v3_command_to_bytes(seq: int, cmd: V3Command | None) -> bytes:
    if cmd is None:
        return encode_stop_downlink(seq, mode_req=MODE_CAN)
    return encode_downlink(
        seq,
        DownlinkCommand(
            mode_req=cmd.mode_req,
            v_mm_s=cmd.v_mm_s,
            omega_millirad_s=cmd.omega_millirad_s,
            steer_millirad=cmd.steer_millirad,
            motion_model=cmd.motion_model,
            light_en=cmd.light_enable,
            light_mode=cmd.light_mode,
            clear_error=cmd.clear_error,
        ),
    )
