"""Publish all BLOB v2 uplink MSG types to /jetson_{link}/blob/* topics."""

from __future__ import annotations

from jetson_mcu_msgs.msg import (
    BlobAgvEnergy,
    BlobAgvMotion,
    BlobGpsCompact,
    BlobMotorGroup,
    BlobMotorPos,
    BlobMcuStatus,
    BlobSensor,
    GpsFrameA,
    GpsFrameB,
    GpsFrameC,
    MotorCompact,
    SonarStamped,
    V3ExtStatus,
    V3Status,
)

from agv_base_driver.blob_codec import (
    MSG_AGV_ENERGY,
    MSG_AGV_MOTION,
    MSG_AGV_MOTOR04,
    MSG_AGV_MOTOR58,
    MSG_AGV_MOTOR_POS,
    MSG_GPS_COMPACT,
    MSG_MCU_STATUS,
    MSG_SENSOR_BLOB,
    AgvEnergy,
    AgvMotion,
    AgvMotorPos,
    BlobUplinkCache,
    GpsCompact,
    McuStatus,
    MotorCompact as MotorCompactDataclass,
    SensorBlob,
    cache_to_v3_ext,
    cache_to_v3_status,
    gps_compact_to_frame_a,
    gps_compact_to_frame_b,
    gps_compact_to_frame_c,
    parse_agv_energy,
    parse_agv_motion,
    parse_agv_motor04,
    parse_agv_motor58,
    parse_agv_motor_pos,
    parse_gps_compact,
    parse_mcu_status,
    parse_sensor_blob,
)
from agv_base_driver.blob_time_stamp import (
    mono_ms,
    mcu_tick_to_ros_stamp,
    mcu_to_jetson_mono_ms,
    mcu_virtual_to_ros_stamp,
)

SONAR_SRC_MCU = 0
SONAR_SRC_SENSOR = 1


def _motor_to_msg(m: MotorCompactDataclass) -> MotorCompact:
    msg = MotorCompact()
    msg.speed_rpm = m.speed_rpm
    msg.current = m.current
    msg.voltage = m.voltage
    msg.temperature = m.temperature
    msg.driver_status = m.driver_status
    msg.position_lo = m.position_lo
    return msg


def _ts_kwargs(offset_ms: float, time_sync_valid: bool) -> dict:
    return {"offset_ms": offset_ms, "time_sync_valid": time_sync_valid}


def motion_to_msg(
    m: AgvMotion,
    stamp,
    blob_seq: int,
    *,
    offset_ms: float = 0.0,
    time_sync_valid: bool = False,
):
    msg = BlobAgvMotion()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.mcu_timestamp_ms = m.timestamp_ms
    msg.blob_seq = blob_seq & 0xFF
    msg.system_info = m.system_info
    msg.motion_info = m.motion_info
    msg.light_pack = m.light_pack
    msg.fault_code = m.fault_code
    msg.battery_voltage_0p1v = m.battery_voltage_0p1v
    msg.linear_velocity = m.linear_velocity
    msg.angular_velocity = m.angular_velocity
    msg.steering_angle = m.steering_angle
    msg.wheel_angle = [int(x) for x in m.wheel_angle]
    msg.wheel_speed = [int(x) for x in m.wheel_speed]
    return msg


def mcu_to_msg(
    m: McuStatus,
    stamp,
    blob_seq: int,
    *,
    offset_ms: float = 0.0,
    time_sync_valid: bool = False,
):
    msg = BlobMcuStatus()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.mcu_timestamp_ms = m.timestamp_ms
    msg.blob_seq = blob_seq & 0xFF
    msg.seq = m.seq
    msg.safety_state = m.safety
    msg.link_state = m.link_flags
    msg.limit_factor = m.limit_factor
    msg.arb_v_mm_s = m.arb_v
    msg.arb_omega_millirad_s = m.arb_w
    msg.arb_steer_millirad = m.arb_steer
    msg.sonar_mm = [int(x) for x in m.sonar_mm]
    msg.sonar_stamp_ms = [int(x) for x in m.sonar_stamp_ms]
    msg.time_sync_valid = time_sync_valid
    if time_sync_valid:
        msg.sonar_jetson_mono_ms = [
            max(0, int(round(mcu_to_jetson_mono_ms(st, offset_ms))))
            for st in m.sonar_stamp_ms
        ]
    else:
        msg.sonar_jetson_mono_ms = [0, 0, 0, 0]
    msg.nearest_mm = m.nearest_mm
    msg.jetson_seq = m.jetson_seq
    return msg


def sensor_to_msg(
    s: SensorBlob,
    stamp,
    blob_seq: int,
    *,
    offset_ms: float = 0.0,
    time_sync_valid: bool = False,
):
    msg = BlobSensor()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.mcu_timestamp_ms = s.timestamp_ms
    msg.blob_seq = blob_seq & 0xFF
    msg.dist_mm = [int(x) for x in s.dist_mm]
    msg.stamp_ms = [int(x) for x in s.stamp_ms]
    msg.time_sync_valid = time_sync_valid
    if time_sync_valid:
        msg.jetson_mono_ms = [
            max(0, int(round(mcu_to_jetson_mono_ms(st, offset_ms))))
            for st in s.stamp_ms
        ]
    else:
        msg.jetson_mono_ms = [0, 0, 0, 0]
    return msg


def gps_to_msg(g: GpsCompact, stamp, blob_seq: int):
    msg = BlobGpsCompact()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.mcu_timestamp_ms = g.timestamp_ms
    msg.blob_seq = blob_seq & 0xFF
    msg.flags = g.flags
    msg.num_sv = g.num_sv
    msg.hdop_x100 = g.hdop_x100
    msg.speed_cm_s = g.speed_cms
    msg.lat_e7 = g.lat_e7
    msg.lon_e7 = g.lon_e7
    msg.heading_x100 = g.heading_x100
    msg.alt_dm = g.alt_dm
    msg.utc_sec = g.utc_sec
    return msg


def energy_to_msg(e: AgvEnergy, stamp, blob_seq: int):
    msg = BlobAgvEnergy()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.mcu_timestamp_ms = e.timestamp_ms
    msg.blob_seq = blob_seq & 0xFF
    msg.odom = [int(x) for x in e.odom]
    msg.bms_soc = e.bms_soc
    msg.bms_soh = e.bms_soh
    msg.bms_voltage_0p1v = e.bms_voltage_0p1v
    msg.bms_current = e.bms_current
    msg.bms_temperature = e.bms_temperature
    msg.bms_alarm1 = e.bms_alarm1
    msg.bms_alarm2 = e.bms_alarm2
    msg.bms_warning1 = e.bms_warning1
    msg.bms_warning2 = e.bms_warning2
    msg.remote = [int(x) for x in e.remote]
    return msg


def motor_group_to_msg(
    motors: list[MotorCompactDataclass],
    timestamp_ms: int,
    stamp,
    blob_seq: int,
    motor_base_index: int,
):
    msg = BlobMotorGroup()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.mcu_timestamp_ms = timestamp_ms
    msg.blob_seq = blob_seq & 0xFF
    msg.motor_base_index = motor_base_index
    msg.motors = [_motor_to_msg(m) for m in motors]
    return msg


def motor_pos_to_msg(p: AgvMotorPos, stamp, blob_seq: int):
    msg = BlobMotorPos()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.mcu_timestamp_ms = p.timestamp_ms
    msg.blob_seq = blob_seq & 0xFF
    msg.motor_position = [int(x) for x in p.motor_position]
    return msg


def build_sonar_stamped(
    dist_mm: list[int],
    mcu_stamps: list[int],
    mcu_frame_ts: int,
    blob_seq: int,
    source: int,
    header_stamp,
    *,
    offset_ms: float,
    time_sync_valid: bool,
) -> SonarStamped:
    msg = SonarStamped()
    msg.header.stamp = header_stamp
    msg.header.frame_id = "stm32b"
    msg.front_mm = int(dist_mm[0])
    msg.back_mm = int(dist_mm[1])
    msg.left_mm = int(dist_mm[2])
    msg.right_mm = int(dist_mm[3])
    msg.mcu_stamp_ms = [int(x) for x in mcu_stamps]
    msg.time_sync_valid = time_sync_valid
    if time_sync_valid:
        msg.jetson_mono_ms = [
            max(0, int(round(mcu_to_jetson_mono_ms(st, offset_ms))))
            for st in mcu_stamps
        ]
    else:
        msg.jetson_mono_ms = [0, 0, 0, 0]
    msg.source = int(source) & 0xFF
    msg.mcu_frame_timestamp_ms = int(mcu_frame_ts)
    msg.blob_seq = blob_seq & 0xFF
    return msg


class BlobTopicPublisher:
    """Shared BLOB uplink cache + ROS publishers for rs232/eth gateway."""

    UPLINK_WATCH_IDS = frozenset(
        {
            MSG_AGV_MOTION,
            MSG_MCU_STATUS,
            MSG_SENSOR_BLOB,
            MSG_GPS_COMPACT,
            MSG_AGV_MOTOR04,
            MSG_AGV_MOTOR58,
            MSG_AGV_ENERGY,
            MSG_AGV_MOTOR_POS,
        }
    )

    def __init__(self, node, topic_prefix: str, *, enable_ros_publish: bool = True) -> None:
        self._node = node
        self._clock = node.get_clock()
        self._prefix = topic_prefix.rstrip("/")
        self._enable_ros_publish = enable_ros_publish
        self.cache = BlobUplinkCache()
        self._motor_ts = {MSG_AGV_MOTOR04: 0, MSG_AGV_MOTOR58: 0}
        self._offset_ms = 0.0
        self._time_sync_valid = False

        p = self._prefix
        if enable_ros_publish:
            self.pub_v3_status = node.create_publisher(V3Status, f"{p}/v3_status", 10)
            self.pub_v3_ext = node.create_publisher(V3ExtStatus, f"{p}/v3_ext_status", 10)
            self.pub_sonar_stamped = node.create_publisher(
                SonarStamped, f"{p}/sonar_stamped", 10
            )
            self.pub_gps_a = node.create_publisher(GpsFrameA, f"{p}/gps/a", 10)
            self.pub_gps_b = node.create_publisher(GpsFrameB, f"{p}/gps/b", 10)
            self.pub_gps_c = node.create_publisher(GpsFrameC, f"{p}/gps/c", 10)
            self.pub_motion = node.create_publisher(BlobAgvMotion, f"{p}/blob/motion", 10)
            self.pub_mcu = node.create_publisher(BlobMcuStatus, f"{p}/blob/mcu_status", 10)
            self.pub_sensor = node.create_publisher(BlobSensor, f"{p}/blob/sensor", 10)
            self.pub_gps = node.create_publisher(BlobGpsCompact, f"{p}/blob/gps", 10)
            self.pub_motor04 = node.create_publisher(
                BlobMotorGroup, f"{p}/blob/motor_04", 10
            )
            self.pub_motor58 = node.create_publisher(
                BlobMotorGroup, f"{p}/blob/motor_58", 10
            )
            self.pub_energy = node.create_publisher(BlobAgvEnergy, f"{p}/blob/energy", 10)
            self.pub_motor_pos = node.create_publisher(
                BlobMotorPos, f"{p}/blob/motor_pos", 10
            )
        else:
            self.pub_v3_status = None
            self.pub_v3_ext = None
            self.pub_sonar_stamped = None
            self.pub_gps_a = None
            self.pub_gps_b = None
            self.pub_gps_c = None
            self.pub_motion = None
            self.pub_mcu = None
            self.pub_sensor = None
            self.pub_gps = None
            self.pub_motor04 = None
            self.pub_motor58 = None
            self.pub_energy = None
            self.pub_motor_pos = None
        self._on_v3_status = None
        self._on_v3_ext = None
        self._on_gps_frames = None

    def _maybe_publish(self, pub, msg) -> None:
        if pub is not None and msg is not None:
            pub.publish(msg)

    def set_uplink_callbacks(
        self,
        *,
        on_v3_status=None,
        on_v3_ext=None,
        on_gps_frames=None,
    ) -> None:
        self._on_v3_status = on_v3_status
        self._on_v3_ext = on_v3_ext
        self._on_gps_frames = on_gps_frames

    def reset_cache(self) -> None:
        self.cache = BlobUplinkCache()
        self._motor_ts = {MSG_AGV_MOTOR04: 0, MSG_AGV_MOTOR58: 0}

    def update_time_sync(self, offset_ms: float, valid: bool) -> None:
        self._offset_ms = offset_ms
        self._time_sync_valid = valid

    def virtual_ros_stamp(self):
        return mcu_virtual_to_ros_stamp(
            self._offset_ms, self._time_sync_valid, self._clock
        )

    def mcu_tx_tick_ms(self) -> int:
        if self._time_sync_valid:
            return int(round(self._offset_ms + mono_ms()))
        return int(mono_ms())

    def _ts_kwargs(self) -> dict:
        return _ts_kwargs(self._offset_ms, self._time_sync_valid)

    def _event_stamp(self, mcu_timestamp_ms: int, rx_stamp=None):
        fallback = rx_stamp if rx_stamp is not None else self.virtual_ros_stamp()
        return mcu_tick_to_ros_stamp(
            mcu_timestamp_ms,
            self._offset_ms,
            self._time_sync_valid,
            self._clock,
            fallback=fallback,
        )

    def _aggregate_stamp(self, rx_stamp=None):
        ts = 0
        if self.cache.mcu is not None:
            ts = self.cache.mcu.timestamp_ms
        elif self.cache.motion is not None:
            ts = self.cache.motion.timestamp_ms
        elif self.cache.sensor is not None:
            ts = self.cache.sensor.timestamp_ms
        if ts > 0:
            return self._event_stamp(ts, rx_stamp)
        if rx_stamp is not None:
            return rx_stamp
        return self.virtual_ros_stamp()

    def handle_frame(self, msg_id: int, blob_seq: int, payload: bytes) -> bool:
        """Parse one BLOB payload, update cache, publish typed topics. Returns uplink active."""
        updated_status = False
        updated_ext = False
        ts_kw = self._ts_kwargs()

        if msg_id == MSG_AGV_MOTION:
            motion = parse_agv_motion(payload)
            if motion is None:
                return False
            self.cache.motion = motion
            ev = self._event_stamp(motion.timestamp_ms)
            self._maybe_publish(
                self.pub_motion, motion_to_msg(motion, ev, blob_seq, **ts_kw)
            )
            updated_status = True
            updated_ext = True
        elif msg_id == MSG_MCU_STATUS:
            mcu = parse_mcu_status(payload)
            if mcu is None:
                return False
            self.cache.mcu = mcu
            ev = self._event_stamp(mcu.timestamp_ms)
            self._maybe_publish(self.pub_mcu, mcu_to_msg(mcu, ev, blob_seq, **ts_kw))
            self._maybe_publish(
                self.pub_sonar_stamped,
                build_sonar_stamped(
                    mcu.sonar_mm,
                    mcu.sonar_stamp_ms,
                    mcu.timestamp_ms,
                    blob_seq,
                    SONAR_SRC_MCU,
                    ev,
                    offset_ms=self._offset_ms,
                    time_sync_valid=self._time_sync_valid,
                )
            )
            updated_status = True
            updated_ext = True
        elif msg_id == MSG_SENSOR_BLOB:
            sensor = parse_sensor_blob(payload)
            if sensor is None:
                return False
            self.cache.sensor = sensor
            ev = self._event_stamp(sensor.timestamp_ms)
            self._maybe_publish(
                self.pub_sensor, sensor_to_msg(sensor, ev, blob_seq, **ts_kw)
            )
            self._maybe_publish(
                self.pub_sonar_stamped,
                build_sonar_stamped(
                    sensor.dist_mm,
                    sensor.stamp_ms,
                    sensor.timestamp_ms,
                    blob_seq,
                    SONAR_SRC_SENSOR,
                    ev,
                    offset_ms=self._offset_ms,
                    time_sync_valid=self._time_sync_valid,
                ),
            )
            updated_status = True
        elif msg_id == MSG_GPS_COMPACT:
            gps = parse_gps_compact(payload)
            if gps is None:
                return False
            ev = self._event_stamp(gps.timestamp_ms)
            self._maybe_publish(self.pub_gps, gps_to_msg(gps, ev, blob_seq))
            fa = gps_compact_to_frame_a(gps, ev)
            fb = gps_compact_to_frame_b(gps, ev)
            fc = gps_compact_to_frame_c(gps, ev)
            self._maybe_publish(self.pub_gps_a, fa)
            self._maybe_publish(self.pub_gps_b, fb)
            self._maybe_publish(self.pub_gps_c, fc)
            if self._on_gps_frames is not None:
                self._on_gps_frames(fa, fb, fc)
        elif msg_id == MSG_AGV_MOTOR04:
            motors = parse_agv_motor04(payload)
            if motors is None:
                return False
            self.cache.update_motors(motors, base=0)
            ts = _u32_ts(payload)
            self._motor_ts[MSG_AGV_MOTOR04] = ts
            ev = self._event_stamp(ts)
            self._maybe_publish(
                self.pub_motor04,
                motor_group_to_msg(motors, ts, ev, blob_seq, motor_base_index=0),
            )
            updated_ext = True
        elif msg_id == MSG_AGV_MOTOR58:
            motors = parse_agv_motor58(payload)
            if motors is None:
                return False
            self.cache.update_motors(motors, base=4)
            ts = _u32_ts(payload)
            self._motor_ts[MSG_AGV_MOTOR58] = ts
            ev = self._event_stamp(ts)
            self._maybe_publish(
                self.pub_motor58,
                motor_group_to_msg(motors, ts, ev, blob_seq, motor_base_index=4),
            )
            updated_ext = True
        elif msg_id == MSG_AGV_ENERGY:
            energy = parse_agv_energy(payload)
            if energy is None:
                return False
            self.cache.energy = energy
            ev = self._event_stamp(energy.timestamp_ms)
            self._maybe_publish(self.pub_energy, energy_to_msg(energy, ev, blob_seq))
            updated_status = True
        elif msg_id == MSG_AGV_MOTOR_POS:
            pos = parse_agv_motor_pos(payload)
            if pos is None:
                return False
            self.cache.motor_pos = pos
            ev = self._event_stamp(pos.timestamp_ms)
            self._maybe_publish(
                self.pub_motor_pos, motor_pos_to_msg(pos, ev, blob_seq)
            )
        else:
            return False

        agg = self._aggregate_stamp()
        if updated_status:
            status_msg = cache_to_v3_status(self.cache, agg)
            if status_msg is not None:
                self._maybe_publish(self.pub_v3_status, status_msg)
                if self._on_v3_status is not None:
                    self._on_v3_status(status_msg)
        if updated_ext:
            ext_msg = cache_to_v3_ext(self.cache, agg)
            if ext_msg is not None:
                self._maybe_publish(self.pub_v3_ext, ext_msg)
                if self._on_v3_ext is not None:
                    self._on_v3_ext(ext_msg)

        return msg_id in self.UPLINK_WATCH_IDS

    def status_publish_count(self) -> int:
        return 1 if self.cache.motion or self.cache.mcu else 0


def _u32_ts(payload: bytes) -> int:
    if len(payload) < 4:
        return 0
    return int.from_bytes(payload[0:4], "big")
