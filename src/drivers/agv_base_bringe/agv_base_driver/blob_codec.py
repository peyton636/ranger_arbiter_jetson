"""BLOB v2 线格式编解码 + 与 V3 ROS 消息映射（见 docs/Jetson_BLOB协议_v2.md）。"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

from jetson_mcu_msgs.msg import GpsFrameA, GpsFrameB, GpsFrameC, V3Command, V3ExtStatus, V3Status

from agv_base_driver.jetson_protocol import MODE_CAN, clamp_i16

BLOB_MAGIC = 0xAB
BLOB_VER = 0x01
BLOB_HDR_LEN = 9

MSG_AGV_CONTROL = 0x01
MSG_AGV_MOTION = 0x02
MSG_MCU_STATUS = 0x03
MSG_SENSOR_BLOB = 0x04
MSG_GPS_COMPACT = 0x05
MSG_AGV_MOTOR04 = 0x06
MSG_AGV_MOTOR58 = 0x07
MSG_AGV_ENERGY = 0x08
MSG_AGV_MOTOR_POS = 0x0B
MSG_SENSOR_CFG = 0x10

PAYLOAD_LEN: dict[int, int] = {
    MSG_AGV_CONTROL: 14,
    MSG_AGV_MOTION: 40,
    MSG_MCU_STATUS: 42,
    MSG_SENSOR_BLOB: 28,
    MSG_GPS_COMPACT: 32,
    MSG_AGV_MOTOR04: 44,
    MSG_AGV_MOTOR58: 44,
    MSG_AGV_ENERGY: 41,
    MSG_AGV_MOTOR_POS: 36,
    MSG_SENSOR_CFG: 8,
}

SONAR_INVALID = 0xFFFF


def _u32be(data: bytes, off: int) -> int:
    return struct.unpack_from(">I", data, off)[0]


def _s16be(data: bytes, off: int) -> int:
    return struct.unpack_from(">h", data, off)[0]


def _u16be(data: bytes, off: int) -> int:
    return struct.unpack_from(">H", data, off)[0]


def _i8(data: bytes, off: int) -> int:
    v = data[off]
    return v - 0x100 if v >= 0x80 else v


def encode_blob_frame(msg_id: int, seq: int, payload: bytes) -> bytes:
    expected = PAYLOAD_LEN.get(msg_id)
    if expected is None or len(payload) != expected:
        raise ValueError(f"BLOB MSG 0x{msg_id:02X} payload 应为 {expected}B，实际 {len(payload)}B")
    hdr = bytearray(BLOB_HDR_LEN)
    hdr[0] = BLOB_MAGIC
    hdr[1] = BLOB_VER
    hdr[2] = msg_id & 0xFF
    hdr[3] = seq & 0xFF
    ln = len(payload)
    hdr[4] = (ln >> 8) & 0xFF
    hdr[5] = ln & 0xFF
    hdr[6] = 0
    hdr[7] = 1
    hdr[8] = 0
    return bytes(hdr) + payload


def encode_agv_control(
    seq: int,
    timestamp_ms: int,
    *,
    control_mode: int = MODE_CAN,
    linear_vel: int = 0,
    angular_vel: int = 0,
    steer_angle: int = 0,
    motion_drive_info: int = 0,
    clear_fault: int = 0,
    light_info: int = 0,
) -> bytes:
    payload = struct.pack(
        ">IhhhBBBB",
        int(timestamp_ms) & 0xFFFFFFFF,
        clamp_i16(linear_vel),
        clamp_i16(angular_vel),
        clamp_i16(steer_angle),
        control_mode & 0xFF,
        motion_drive_info & 0xFF,
        clear_fault & 0xFF,
        light_info & 0xFF,
    )
    return encode_blob_frame(MSG_AGV_CONTROL, seq, payload)


def v3_command_to_blob(seq: int, cmd: V3Command | None, timestamp_ms: int) -> bytes:
    if cmd is None:
        return encode_agv_control(seq, timestamp_ms, control_mode=MODE_CAN)
    light_info = ((cmd.light_mode & 0x01) << 1) | (cmd.light_enable & 0x01)
    return encode_agv_control(
        seq,
        timestamp_ms,
        control_mode=cmd.mode_req,
        linear_vel=cmd.v_mm_s,
        angular_vel=cmd.omega_millirad_s,
        steer_angle=cmd.steer_millirad,
        motion_drive_info=cmd.motion_model,
        clear_fault=cmd.clear_error,
        light_info=light_info,
    )


@dataclass
class AgvMotion:
    timestamp_ms: int = 0
    system_info: int = 0
    motion_info: int = 0
    light_pack: int = 0
    fault_code: int = 0
    battery_voltage_0p1v: int = 0
    linear_velocity: int = 0
    angular_velocity: int = 0
    steering_angle: int = 0
    wheel_angle: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    wheel_speed: list[int] = field(default_factory=lambda: [0, 0, 0, 0])


@dataclass
class McuStatus:
    timestamp_ms: int = 0
    seq: int = 0
    safety: int = 0
    link_flags: int = 0
    limit_factor: int = 0
    arb_v: int = 0
    arb_w: int = 0
    arb_steer: int = 0
    sonar_mm: list[int] = field(default_factory=lambda: [SONAR_INVALID] * 4)
    sonar_stamp_ms: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    nearest_mm: int = SONAR_INVALID
    jetson_seq: int = 0


@dataclass
class SensorBlob:
    timestamp_ms: int = 0
    dist_mm: list[int] = field(default_factory=lambda: [SONAR_INVALID] * 4)
    stamp_ms: list[int] = field(default_factory=lambda: [0, 0, 0, 0])


@dataclass
class MotorCompact:
    speed_rpm: int = 0
    current: int = 0
    voltage: int = 0
    temperature: int = 0
    driver_status: int = 0
    position_lo: int = 0


@dataclass
class AgvEnergy:
    timestamp_ms: int = 0
    odom: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    bms_soc: int = 0
    bms_soh: int = 0
    bms_voltage_0p1v: int = 0
    bms_current: int = 0
    bms_temperature: int = 0
    bms_alarm1: int = 0
    bms_alarm2: int = 0
    bms_warning1: int = 0
    bms_warning2: int = 0
    remote: list[int] = field(default_factory=lambda: [0] * 7)


@dataclass
class AgvMotorPos:
    timestamp_ms: int = 0
    motor_position: list[int] = field(default_factory=lambda: [0] * 8)


@dataclass
class GpsCompact:
    timestamp_ms: int = 0
    flags: int = 0
    num_sv: int = 0
    hdop_x100: int = 0xFFFF
    speed_cms: int = 0
    lat_e7: int = 0
    lon_e7: int = 0
    heading_x100: int = 0
    alt_dm: int = 0x7FFF
    utc_sec: int = 0


@dataclass
class BlobUplinkCache:
    motion: AgvMotion | None = None
    mcu: McuStatus | None = None
    sensor: SensorBlob | None = None
    energy: AgvEnergy | None = None
    motor_pos: AgvMotorPos | None = None
    motors: list[MotorCompact] = field(default_factory=list)

    def update_motors(self, chunk: list[MotorCompact], base: int = 0) -> None:
        while len(self.motors) < 8:
            self.motors.append(MotorCompact())
        for i, m in enumerate(chunk):
            self.motors[base + i] = m


def parse_agv_motion(payload: bytes) -> AgvMotion | None:
    if len(payload) != PAYLOAD_LEN[MSG_AGV_MOTION]:
        return None
    m = AgvMotion()
    m.timestamp_ms = _u32be(payload, 0)
    m.system_info = payload[4]
    m.motion_info = payload[5]
    m.light_pack = payload[6]
    m.fault_code = _u32be(payload, 7)
    m.battery_voltage_0p1v = _u16be(payload, 11)
    m.linear_velocity = _s16be(payload, 13)
    m.angular_velocity = _s16be(payload, 15)
    m.steering_angle = _s16be(payload, 17)
    for i in range(4):
        m.wheel_angle[i] = _s16be(payload, 19 + i * 2)
        m.wheel_speed[i] = _s16be(payload, 27 + i * 2)
    return m


def parse_mcu_status(payload: bytes) -> McuStatus | None:
    if len(payload) != PAYLOAD_LEN[MSG_MCU_STATUS]:
        return None
    s = McuStatus()
    s.timestamp_ms = _u32be(payload, 0)
    s.seq = payload[4]
    s.safety = payload[5]
    s.link_flags = payload[6]
    s.limit_factor = payload[7]
    s.arb_v = _s16be(payload, 8)
    s.arb_w = _s16be(payload, 10)
    s.arb_steer = _s16be(payload, 12)
    for i in range(4):
        s.sonar_mm[i] = _u16be(payload, 14 + i * 2)
        s.sonar_stamp_ms[i] = _u32be(payload, 22 + i * 4)
    s.nearest_mm = _u16be(payload, 38)
    s.jetson_seq = payload[40]
    return s


def parse_sensor_blob(payload: bytes) -> SensorBlob | None:
    if len(payload) != PAYLOAD_LEN[MSG_SENSOR_BLOB]:
        return None
    b = SensorBlob()
    b.timestamp_ms = _u32be(payload, 0)
    for i in range(4):
        b.dist_mm[i] = _u16be(payload, 4 + i * 2)
        b.stamp_ms[i] = _u32be(payload, 12 + i * 4)
    return b


def _parse_motor_chunk(payload: bytes) -> list[MotorCompact] | None:
    if len(payload) != 44:
        return None
    motors: list[MotorCompact] = []
    for i in range(4):
        off = 4 + i * 10
        motors.append(
            MotorCompact(
                speed_rpm=_s16be(payload, off),
                current=_s16be(payload, off + 2),
                voltage=_u16be(payload, off + 4),
                temperature=_i8(payload, off + 6),
                driver_status=payload[off + 7],
                position_lo=_u16be(payload, off + 8),
            )
        )
    return motors


def parse_agv_motor04(payload: bytes) -> list[MotorCompact] | None:
    if len(payload) != PAYLOAD_LEN[MSG_AGV_MOTOR04]:
        return None
    return _parse_motor_chunk(payload)


def parse_agv_motor58(payload: bytes) -> list[MotorCompact] | None:
    if len(payload) != PAYLOAD_LEN[MSG_AGV_MOTOR58]:
        return None
    return _parse_motor_chunk(payload)


def parse_gps_compact(payload: bytes) -> GpsCompact | None:
    if len(payload) != PAYLOAD_LEN[MSG_GPS_COMPACT]:
        return None
    g = GpsCompact()
    g.timestamp_ms = _u32be(payload, 0)
    g.flags = payload[4]
    g.num_sv = payload[5]
    g.hdop_x100 = _u16be(payload, 6)
    g.speed_cms = _u16be(payload, 8)
    g.lat_e7 = struct.unpack_from(">i", payload, 10)[0]
    g.lon_e7 = struct.unpack_from(">i", payload, 14)[0]
    g.heading_x100 = _s16be(payload, 18)
    g.alt_dm = _s16be(payload, 20)
    g.utc_sec = _u32be(payload, 22)
    return g


def gps_compact_to_frame_a(gps: GpsCompact, stamp) -> GpsFrameA:
    msg = GpsFrameA()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.magic = 0xA4
    msg.frag_idx = 0
    msg.flags = gps.flags
    msg.num_sv = gps.num_sv
    msg.hdop_x100 = gps.hdop_x100
    msg.speed_cm_s = gps.speed_cms
    return msg


def gps_compact_to_frame_b(gps: GpsCompact, stamp) -> GpsFrameB:
    msg = GpsFrameB()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.magic = 0xA4
    msg.frag_idx = 1
    msg.lat_e7 = gps.lat_e7
    msg.heading_x100 = gps.heading_x100
    return msg


def gps_compact_to_frame_c(gps: GpsCompact, stamp) -> GpsFrameC:
    msg = GpsFrameC()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.magic = 0xA4
    msg.frag_idx = 2
    msg.lon_e7 = gps.lon_e7
    msg.alt_dm = gps.alt_dm
    return msg


def parse_agv_energy(payload: bytes) -> AgvEnergy | None:
    if len(payload) != PAYLOAD_LEN[MSG_AGV_ENERGY]:
        return None
    e = AgvEnergy()
    e.timestamp_ms = _u32be(payload, 0)
    for i in range(4):
        e.odom[i] = struct.unpack_from(">i", payload, 4 + i * 4)[0]
    e.bms_soc = payload[20]
    e.bms_soh = payload[21]
    e.bms_voltage_0p1v = _u16be(payload, 22)
    e.bms_current = _s16be(payload, 24)
    e.bms_temperature = _s16be(payload, 26)
    e.bms_alarm1 = payload[28]
    e.bms_alarm2 = payload[29]
    e.bms_warning1 = payload[30]
    e.bms_warning2 = payload[31]
    e.remote = list(payload[32:39])
    return e


def parse_agv_motor_pos(payload: bytes) -> AgvMotorPos | None:
    if len(payload) != PAYLOAD_LEN[MSG_AGV_MOTOR_POS]:
        return None
    p = AgvMotorPos()
    p.timestamp_ms = _u32be(payload, 0)
    for i in range(8):
        p.motor_position[i] = struct.unpack_from(">i", payload, 4 + i * 4)[0]
    return p


def encode_sensor_cfg(
    seq: int,
    timestamp_ms: int,
    *,
    threshold_mm: int = 0,
    enable_mask: int = 0x0F,
) -> bytes:
    payload = struct.pack(
        ">IHB",
        int(timestamp_ms) & 0xFFFFFFFF,
        threshold_mm & 0xFFFF,
        enable_mask & 0xFF,
    ) + bytes([0])
    return encode_blob_frame(MSG_SENSOR_CFG, seq, payload)


def _sonar_u16(raw: int) -> int:
    if raw == SONAR_INVALID or raw == 0:
        return SONAR_INVALID
    return raw


def cache_to_v3_status(cache: BlobUplinkCache, stamp) -> V3Status | None:
    if cache.motion is None and cache.mcu is None:
        return None
    msg = V3Status()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"

    if cache.mcu is not None:
        msg.seq = cache.mcu.seq
        msg.safety_state = cache.mcu.safety
        msg.link_state = cache.mcu.link_flags
        msg.limit_factor = cache.mcu.limit_factor
        msg.sonar_front_mm = _sonar_u16(cache.mcu.sonar_mm[0])
        msg.sonar_back_mm = _sonar_u16(cache.mcu.sonar_mm[1])
        msg.sonar_left_mm = _sonar_u16(cache.mcu.sonar_mm[2])
        msg.sonar_right_mm = _sonar_u16(cache.mcu.sonar_mm[3])

    if cache.sensor is not None:
        msg.sonar_front_mm = _sonar_u16(cache.sensor.dist_mm[0])
        msg.sonar_back_mm = _sonar_u16(cache.sensor.dist_mm[1])
        msg.sonar_left_mm = _sonar_u16(cache.sensor.dist_mm[2])
        msg.sonar_right_mm = _sonar_u16(cache.sensor.dist_mm[3])

    if cache.motion is not None:
        msg.fb_v_mm_s = cache.motion.linear_velocity
        msg.fb_omega_millirad_s = cache.motion.angular_velocity
        msg.fb_steer_millirad = cache.motion.steering_angle
        msg.battery_voltage_0p1v = cache.motion.battery_voltage_0p1v

    if cache.energy is not None:
        msg.battery_soc = cache.energy.bms_soc
        if cache.energy.bms_voltage_0p1v:
            msg.battery_voltage_0p1v = cache.energy.bms_voltage_0p1v

    return msg


def cache_to_v3_ext(cache: BlobUplinkCache, stamp) -> V3ExtStatus | None:
    if cache.motion is None and len(cache.motors) < 4:
        return None
    msg = V3ExtStatus()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.seq = cache.mcu.seq if cache.mcu else 0

    if cache.motion is not None:
        ws = cache.motion.wheel_speed
        wa = cache.motion.wheel_angle
        msg.wheel_rf = ws[0]
        msg.wheel_rr = ws[1]
        msg.wheel_lr = ws[2]
        msg.wheel_lf = ws[3]
        msg.steer_rf_millirad = wa[0]
        msg.steer_rr_millirad = wa[1]
        msg.steer_lr_millirad = wa[2]
        msg.steer_lf_millirad = wa[3]

    if cache.motors:
        temps = [m.temperature for m in cache.motors if m.temperature != 0]
        msg.motor_temp_max_c = max(temps) if temps else 0
        drv = 0
        for m in cache.motors:
            drv |= m.driver_status
        msg.driver_state_or = drv

    return msg
