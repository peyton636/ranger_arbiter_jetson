"""Jetson ↔ STM32B protocol V3.0 — 24-byte frames (see ~/catkin_ws/docs/PROTOCOL_V3.md)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

FRAME_HEADER = 0xAA
FRAME_LEN = 24

FRAME_TYPE_DOWN = 0x01
FRAME_TYPE_UP_STATUS = 0x02
FRAME_TYPE_UP_EXT = 0x03

MODE_STANDBY = 0
MODE_CAN = 1
MODE_REMOTE = 2

MOTION_ACKERMANN = 0
MOTION_SIDEWAYS = 1
MOTION_SPIN = 2
MOTION_PARK = 3

MAX_V_MM_S = 2000
MAX_V_MM_S_LARGE_STEER = 700  # |steer| > 20° in 0x111 (Ranger manual)
MAX_OMEGA_MILLIRAD_S = 3259  # 0.001 rad/s per count
MAX_STEER_ACKERMANN_MILLIRAD = 698
MAX_STEER_SIDEWAYS_MILLIRAD = 1571  # 90° crab / 斜移 (Ranger Mini 3.0 manual)
# Back-compat alias
MAX_STEER_MILLIRAD = MAX_STEER_ACKERMANN_MILLIRAD
STEER_LARGE_ANGLE_MILLIRAD = 350  # ~20°

INVALID_SONAR_MM = 0xFFFF

RECOVER_STABLE_MS = 1000
CMD_TIMEOUT_MS = 200
UPLINK_TIMEOUT_MS = 300

SAFETY_NAMES = {
    1: "NORMAL",
    2: "SPEED_LIMIT",
    3: "DEGRADED",
    4: "EMERGENCY",
}

SAFETY_NAMES_ZH = {
    1: "正常",
    2: "限速避障",
    3: "降级(断连自救)",
    4: "紧急停车",
}


def clamp_i16(value: int, lo: int = -32768, hi: int = 32767) -> int:
    return max(lo, min(hi, value))


def xor_frame(data: bytes) -> int:
    cs = 0
    for b in data[:23]:
        cs ^= b
    return cs


def u16_be(data: bytes, index: int) -> int:
    return (data[index] << 8) | data[index + 1]


def s16_be(data: bytes, index: int) -> int:
    raw = u16_be(data, index)
    return raw - 0x10000 if raw >= 0x8000 else raw


def twist_to_cmd(linear_x_m_s: float, angular_z_rad_s: float) -> tuple[int, int]:
    """ROS Twist -> (v_mm_s, omega_millirad_s) for ackermann / spin."""
    v_mm_s = clamp_i16(int(round(linear_x_m_s * 1000.0)), -MAX_V_MM_S, MAX_V_MM_S)
    omega = clamp_i16(
        int(round(angular_z_rad_s * 1000.0)),
        -MAX_OMEGA_MILLIRAD_S,
        MAX_OMEGA_MILLIRAD_S,
    )
    return v_mm_s, omega


def _clamp_v_for_steer(v_mm_s: int, steer_millirad: int) -> int:
    if abs(steer_millirad) > STEER_LARGE_ANGLE_MILLIRAD:
        return clamp_i16(v_mm_s, -MAX_V_MM_S_LARGE_STEER, MAX_V_MM_S_LARGE_STEER)
    return clamp_i16(v_mm_s, -MAX_V_MM_S, MAX_V_MM_S)


@dataclass
class MotionCommand:
    """Fields for one V3 downlink motion tick."""

    v_mm_s: int = 0
    omega_millirad_s: int = 0
    steer_millirad: int = 0
    motion_model: int = MOTION_ACKERMANN


def twist_to_motion(
    linear_x_m_s: float,
    linear_y_m_s: float,
    angular_z_rad_s: float,
    *,
    strafe_jl_from_angular: bool = True,
    strafe_speed_m_s: float = 0.3,
    sideways_steer_millirad: int = MAX_STEER_SIDEWAYS_MILLIRAD,
) -> MotionCommand:
    """
    Map ROS /cmd_vel to Ranger CAN semantics (via V3).

    - linear.y or (strafe_jl + only angular.z): 斜移 — motion_model=1, |steer|≈90°,
      v = speed (manual: right stick 90° = lateral).
    - linear.x + angular.z: 阿克曼 — v + ω.
    - only angular.z without strafe_jl: 自旋 — motion_model=2.
    """
    eps = 0.02
    lx, ly, az = linear_x_m_s, linear_y_m_s, angular_z_rad_s
    steer_cap = clamp_i16(
        sideways_steer_millirad, -MAX_STEER_SIDEWAYS_MILLIRAD, MAX_STEER_SIDEWAYS_MILLIRAD
    )

    if abs(ly) > eps:
        steer = steer_cap if ly > 0 else -steer_cap
        v_mm_s = _clamp_v_for_steer(int(round(abs(ly) * 1000.0)), steer)
        return MotionCommand(
            v_mm_s=v_mm_s,
            omega_millirad_s=0,
            steer_millirad=steer,
            motion_model=MOTION_SIDEWAYS,
        )

    if strafe_jl_from_angular and abs(lx) <= eps and abs(az) > eps:
        steer = steer_cap if az > 0 else -steer_cap
        v_mm_s = _clamp_v_for_steer(int(round(strafe_speed_m_s * 1000.0)), steer)
        return MotionCommand(
            v_mm_s=v_mm_s,
            omega_millirad_s=0,
            steer_millirad=steer,
            motion_model=MOTION_SIDEWAYS,
        )

    if abs(lx) <= eps and abs(az) > eps:
        v_mm_s, omega = twist_to_cmd(0.0, az)
        return MotionCommand(
            v_mm_s=v_mm_s,
            omega_millirad_s=omega,
            steer_millirad=0,
            motion_model=MOTION_SPIN,
        )

    v_mm_s, omega = twist_to_cmd(lx, az)
    return MotionCommand(
        v_mm_s=v_mm_s,
        omega_millirad_s=omega,
        steer_millirad=0,
        motion_model=MOTION_ACKERMANN,
    )


def pack_s16_be(value: int) -> tuple[int, int]:
    value = clamp_i16(value)
    if value < 0:
        value += 0x10000
    return (value >> 8) & 0xFF, value & 0xFF


@dataclass
class DownlinkCommand:
    mode_req: int = MODE_CAN
    v_mm_s: int = 0
    omega_millirad_s: int = 0
    steer_millirad: int = 0
    motion_model: int = MOTION_ACKERMANN
    light_en: int = 0
    light_mode: int = 0
    clear_error: int = 0


@dataclass
class UplinkStatus:
    seq: int
    safety_state: int
    link_state: int
    limit_factor: int
    v_actual_mm_s: int
    omega_millirad_s: int
    steer_millirad: int
    sonar_front_mm: int | None
    sonar_back_mm: int | None
    sonar_left_mm: int | None
    sonar_right_mm: int | None
    battery_voltage_0p1v: int
    battery_soc: int

    @property
    def jetson_link_lost(self) -> bool:
        return bool(self.link_state & 0x01)

    @property
    def can_link_lost(self) -> bool:
        return bool(self.link_state & 0x02)

    @property
    def safety_name(self) -> str:
        return SAFETY_NAMES.get(self.safety_state, f"UNKNOWN({self.safety_state})")


def _sonar_mm(raw: int) -> int | None:
    if raw == INVALID_SONAR_MM or raw == 0:
        return None
    return raw


def encode_downlink(seq: int, cmd: DownlinkCommand) -> bytes:
    """Build 24-byte Jetson -> STM32B frame (frame_type=0x01)."""
    frame = bytearray(FRAME_LEN)
    frame[0] = FRAME_HEADER
    frame[1] = FRAME_TYPE_DOWN
    frame[2] = seq & 0xFF
    frame[3] = cmd.mode_req & 0xFF

    hi, lo = pack_s16_be(cmd.v_mm_s)
    frame[4], frame[5] = hi, lo
    hi, lo = pack_s16_be(cmd.omega_millirad_s)
    frame[6], frame[7] = hi, lo
    hi, lo = pack_s16_be(cmd.steer_millirad)
    frame[8], frame[9] = hi, lo

    frame[10] = cmd.motion_model & 0xFF
    frame[11] = cmd.light_en & 0xFF
    frame[12] = cmd.light_mode & 0xFF
    frame[13] = cmd.clear_error & 0xFF
    # 14~22 reserved = 0

    frame[23] = xor_frame(frame)
    return bytes(frame)


def encode_stop_downlink(seq: int, mode_req: int = MODE_CAN) -> bytes:
    return encode_downlink(
        seq,
        DownlinkCommand(mode_req=mode_req, v_mm_s=0, omega_millirad_s=0),
    )


_MODE_REQ_NAMES = {0: "STANDBY", 1: "CAN", 2: "REMOTE"}
_MOTION_MODEL_NAMES = {
    0: "ACKERMANN",
    1: "SIDEWAYS",
    2: "SPIN",
    3: "PARK",
}


def parse_downlink_fields(frame: bytes) -> dict[str, int] | None:
    """Decode v/ω/steer from a 24-byte downlink frame (for logging)."""
    if len(frame) != FRAME_LEN:
        return None
    if frame[0] != FRAME_HEADER or frame[1] != FRAME_TYPE_DOWN:
        return None
    if xor_frame(frame) != frame[23]:
        return None
    return {
        "seq": frame[2],
        "mode_req": frame[3],
        "v_mm_s": s16_be(frame, 4),
        "omega_millirad_s": s16_be(frame, 6),
        "steer_millirad": s16_be(frame, 8),
        "motion_model": frame[10],
    }


def format_downlink_log(frame: bytes, bridge_mode: str = "") -> str:
    """Human-readable TX summary (Jetson -> STM32B)."""
    f = parse_downlink_fields(frame)
    if f is None:
        return f"无效下行帧 hex={frame.hex()}"
    mode_name = _MODE_REQ_NAMES.get(f["mode_req"], str(f["mode_req"]))
    motion_name = _MOTION_MODEL_NAMES.get(
        f.get("motion_model", 0), str(f.get("motion_model", "?"))
    )
    omega_rad_s = f["omega_millirad_s"] / 1000.0
    extra = f" bridge={bridge_mode}" if bridge_mode else ""
    return (
        f"下发 seq={f['seq']} v={f['v_mm_s']}mm/s "
        f"ω={f['omega_millirad_s']}(={omega_rad_s:.3f}rad/s) "
        f"steer={f['steer_millirad']} motion={motion_name} mode_req={mode_name}{extra}"
    )


def format_uplink_log(status: UplinkStatus) -> str:
    """Human-readable RX summary (STM32B -> Jetson, chassis feedback)."""
    omega_rad_s = status.omega_millirad_s / 1000.0
    return (
        f"上行 仲裁={status.safety_name} limit={status.limit_factor}% "
        f"底盘实际v={status.v_actual_mm_s}mm/s "
        f"ω={status.omega_millirad_s}(={omega_rad_s:.3f}rad/s) "
        f"link=0x{status.link_state:02x}"
        f"{' jetson_lost' if status.jetson_link_lost else ''}"
    )


def _sonar_mm_label(v: int | None) -> str:
    if v is None:
        return "无效"
    return str(v)


def format_status_echo(status: UplinkStatus) -> str:
    """Chinese one-line summary for `ros2 topic echo /stm32b/status`."""
    safety_zh = SAFETY_NAMES_ZH.get(
        status.safety_state, f"未知({status.safety_state})"
    )
    b_jetson_link = "断开" if status.jetson_link_lost else "正常"
    can_link = "断开" if status.can_link_lost else "正常"

    return (
        f"Jetson串口=已连接 "
        f"仲裁={safety_zh}({status.safety_name}) "
        f"限速={status.limit_factor}% "
        f"B板报告Jetson心跳={b_jetson_link} CAN链路={can_link} "
        f"底盘v={status.v_actual_mm_s}mm/s "
        f"超声[前{_sonar_mm_label(status.sonar_front_mm)} "
        f"后{_sonar_mm_label(status.sonar_back_mm)} "
        f"左{_sonar_mm_label(status.sonar_left_mm)} "
        f"右{_sonar_mm_label(status.sonar_right_mm)}]mm "
        f"电池={status.battery_voltage_0p1v / 10.0:.1f}V "
        f"SOC={status.battery_soc}%"
    )


def format_bridge_local_status(
    *,
    serial_ok: bool,
    waiting_uplink: bool = False,
    uplink_stale: bool = False,
) -> str:
    """Status when Jetson-side serial is down or STM32B uplink missing."""
    if not serial_ok:
        return (
            "Jetson串口=断开 USB未连接或重连中 "
            "请插回线缆并等3~5秒 (无STM32B实时数据)"
        )
    if waiting_uplink:
        return (
            "Jetson串口=已连接 STM32B上行=无 "
            "已发TX等待0x02 若超过5秒仍无RX请复位B板或重插USB"
        )
    if uplink_stale:
        return (
            "Jetson串口=已连接 STM32B上行=超时(>300ms) "
            "请检查B板固件/串口线"
        )
    return "Jetson串口=已连接 等待STM32B数据"


def parse_uplink_status(frame: bytes) -> UplinkStatus | None:
    if len(frame) != FRAME_LEN:
        return None
    if frame[0] != FRAME_HEADER or frame[1] != FRAME_TYPE_UP_STATUS:
        return None
    if xor_frame(frame) != frame[23]:
        return None

    return UplinkStatus(
        seq=frame[2],
        safety_state=frame[3],
        link_state=frame[4],
        limit_factor=frame[5],
        v_actual_mm_s=s16_be(frame, 6),
        omega_millirad_s=s16_be(frame, 8),
        steer_millirad=s16_be(frame, 10),
        sonar_front_mm=_sonar_mm(u16_be(frame, 12)),
        sonar_back_mm=_sonar_mm(u16_be(frame, 14)),
        sonar_left_mm=_sonar_mm(u16_be(frame, 16)),
        sonar_right_mm=_sonar_mm(u16_be(frame, 18)),
        battery_voltage_0p1v=u16_be(frame, 20),
        battery_soc=frame[22],
    )


@dataclass
class UplinkExt:
    seq: int
    wheel_rf: int
    wheel_rr: int
    wheel_lr: int
    wheel_lf: int
    steer_rf_millirad: int
    steer_rr_millirad: int
    steer_lr_millirad: int
    steer_lf_millirad: int
    motor_temp_max_c: int
    driver_state_or: int


def parse_uplink_ext(frame: bytes) -> UplinkExt | None:
    if len(frame) != FRAME_LEN:
        return None
    if frame[0] != FRAME_HEADER or frame[1] != FRAME_TYPE_UP_EXT:
        return None
    if xor_frame(frame) != frame[23]:
        return None
    temp = frame[19]
    if temp >= 0x80:
        temp -= 0x100
    return UplinkExt(
        seq=frame[2],
        wheel_rf=s16_be(frame, 3),
        wheel_rr=s16_be(frame, 5),
        wheel_lr=s16_be(frame, 7),
        wheel_lf=s16_be(frame, 9),
        steer_rf_millirad=s16_be(frame, 11),
        steer_rr_millirad=s16_be(frame, 13),
        steer_lr_millirad=s16_be(frame, 15),
        steer_lf_millirad=s16_be(frame, 17),
        motor_temp_max_c=temp,
        driver_state_or=frame[20],
    )


class FrameParser:
    """Extract valid 24-byte V3 frames from a serial byte stream."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> Iterator[bytes]:
        self._buf.extend(data)
        while len(self._buf) >= FRAME_LEN:
            if self._buf[0] != FRAME_HEADER:
                self._buf.pop(0)
                continue
            if self._buf[1] not in (
                FRAME_TYPE_DOWN,
                FRAME_TYPE_UP_STATUS,
                FRAME_TYPE_UP_EXT,
            ):
                self._buf.pop(0)
                continue

            frame = bytes(self._buf[:FRAME_LEN])
            del self._buf[:FRAME_LEN]
            if xor_frame(frame) != frame[23]:
                continue
            yield frame
