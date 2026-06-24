"""0xA5 服务帧 8B 载荷 ↔ jetson_mcu_msgs。"""

from __future__ import annotations

import struct

from jetson_mcu_msgs.msg import (
    FaultReport,
    GpsFrameA,
    GpsFrameB,
    GpsFrameC,
    StatusSnapshot,
    TimeSyncResponse,
)

from agv_base_driver.jetson_protocol import s16_be, u16_be


def _u32_be(data: bytes, index: int) -> int:
    return (
        (data[index] << 24)
        | (data[index + 1] << 16)
        | (data[index + 2] << 8)
        | data[index + 3]
    )


def payload_to_gps_a(payload: bytes, stamp) -> GpsFrameA | None:
    if len(payload) < 8 or payload[0] != 0xA4:
        return None
    msg = GpsFrameA()
    msg.header.stamp = stamp
    msg.header.frame_id = "gps"
    msg.magic = payload[0]
    msg.frag_idx = payload[1]
    msg.flags = payload[2]
    msg.num_sv = payload[3]
    msg.hdop_x100 = u16_be(payload, 4)
    msg.speed_cm_s = u16_be(payload, 6)
    return msg


def payload_to_gps_b(payload: bytes, stamp) -> GpsFrameB | None:
    if len(payload) < 8 or payload[0] != 0xA4:
        return None
    msg = GpsFrameB()
    msg.header.stamp = stamp
    msg.header.frame_id = "gps"
    msg.magic = payload[0]
    msg.frag_idx = payload[1]
    msg.lat_e7 = struct.unpack(">i", payload[2:6])[0]
    msg.heading_x100 = s16_be(payload, 6)
    return msg


def payload_to_gps_c(payload: bytes, stamp) -> GpsFrameC | None:
    if len(payload) < 8 or payload[0] != 0xA4:
        return None
    msg = GpsFrameC()
    msg.header.stamp = stamp
    msg.header.frame_id = "gps"
    msg.magic = payload[0]
    msg.frag_idx = payload[1]
    msg.lon_e7 = struct.unpack(">i", payload[2:6])[0]
    msg.alt_dm = s16_be(payload, 6)
    return msg


def payload_to_time_sync(
    payload: bytes,
    stamp,
    update: object | None = None,
) -> TimeSyncResponse:
    msg = TimeSyncResponse()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    if update is not None:
        msg.system_tick_ms = int(getattr(update, "system_tick_ms", 0))
        msg.utc_unix_sec = int(getattr(update, "utc_unix_sec", 0))
        msg.cmd_echo = int(getattr(update, "cmd_echo", 0))
        msg.seq_echo = int(getattr(update, "seq_echo", 0))
        msg.rtt_ms = float(getattr(update, "rtt_ms", 0.0))
        msg.rtt_ema_ms = float(getattr(update, "rtt_ema_ms", 0.0))
        msg.offset_ms = float(getattr(update, "offset_ms", 0.0))
        msg.proc_ms = float(getattr(update, "proc_ms", 0.0))
        msg.gps_utc_valid = bool(getattr(update, "gps_utc_valid", False))
        msg.offset_valid = bool(getattr(update, "offset_valid", False))
        msg.sample_accepted = bool(getattr(update, "sample_accepted", False))
        return msg
    msg.system_tick_ms = _u32_be(payload, 0)
    msg.utc_unix_sec = _u32_be(payload, 4)
    return msg


def payload_to_fault(payload: bytes, stamp) -> FaultReport:
    msg = FaultReport()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.fault_code_1 = payload[0]
    msg.fault_code_2 = payload[1]
    msg.fault_code_3 = payload[2]
    msg.fault_code_4 = payload[3]
    msg.fault_ts_ms = u16_be(payload, 4)
    msg.chassis_sys = payload[6]
    msg.chassis_fault_b0 = payload[7]
    return msg


def payload_to_status_snapshot(payload: bytes, stamp) -> StatusSnapshot:
    msg = StatusSnapshot()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.safety_state = payload[0]
    msg.link_state = payload[1]
    msg.fb_v_mm_s = s16_be(payload, 2)
    msg.nearest_mm = u16_be(payload, 4)
    msg.limit_factor = payload[6]
    msg.flags = payload[7]
    return msg
