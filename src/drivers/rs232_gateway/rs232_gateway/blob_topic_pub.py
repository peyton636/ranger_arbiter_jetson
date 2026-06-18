"""Publish all BLOB v2 uplink MSG types to /jetson_{link}/blob/* topics."""

from __future__ import annotations

from jetson_can_msgs.msg import (
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
    V3ExtStatus,
    V3Status,
)

from rs232_gateway.blob_codec import (
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


def _motor_to_msg(m: MotorCompactDataclass) -> MotorCompact:
    msg = MotorCompact()
    msg.speed_rpm = m.speed_rpm
    msg.current = m.current
    msg.voltage = m.voltage
    msg.temperature = m.temperature
    msg.driver_status = m.driver_status
    msg.position_lo = m.position_lo
    return msg


def motion_to_msg(m: AgvMotion, stamp, blob_seq: int):
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


def mcu_to_msg(m: McuStatus, stamp, blob_seq: int):
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
    msg.nearest_mm = m.nearest_mm
    msg.jetson_seq = m.jetson_seq
    return msg


def sensor_to_msg(s: SensorBlob, stamp, blob_seq: int):
    msg = BlobSensor()
    msg.header.stamp = stamp
    msg.header.frame_id = "stm32b"
    msg.mcu_timestamp_ms = s.timestamp_ms
    msg.blob_seq = blob_seq & 0xFF
    msg.dist_mm = [int(x) for x in s.dist_mm]
    msg.stamp_ms = [int(x) for x in s.stamp_ms]
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

    def __init__(self, node, topic_prefix: str) -> None:
        self._node = node
        self._prefix = topic_prefix.rstrip("/")
        self.cache = BlobUplinkCache()
        self._motor_ts = {MSG_AGV_MOTOR04: 0, MSG_AGV_MOTOR58: 0}

        p = self._prefix
        self.pub_v3_status = node.create_publisher(V3Status, f"{p}/v3_status", 10)
        self.pub_v3_ext = node.create_publisher(V3ExtStatus, f"{p}/v3_ext_status", 10)
        self.pub_gps_a = node.create_publisher(GpsFrameA, f"{p}/gps/a", 10)
        self.pub_gps_b = node.create_publisher(GpsFrameB, f"{p}/gps/b", 10)
        self.pub_gps_c = node.create_publisher(GpsFrameC, f"{p}/gps/c", 10)
        self.pub_motion = node.create_publisher(BlobAgvMotion, f"{p}/blob/motion", 10)
        self.pub_mcu = node.create_publisher(BlobMcuStatus, f"{p}/blob/mcu_status", 10)
        self.pub_sensor = node.create_publisher(BlobSensor, f"{p}/blob/sensor", 10)
        self.pub_gps = node.create_publisher(BlobGpsCompact, f"{p}/blob/gps", 10)
        self.pub_motor04 = node.create_publisher(BlobMotorGroup, f"{p}/blob/motor_04", 10)
        self.pub_motor58 = node.create_publisher(BlobMotorGroup, f"{p}/blob/motor_58", 10)
        self.pub_energy = node.create_publisher(BlobAgvEnergy, f"{p}/blob/energy", 10)
        self.pub_motor_pos = node.create_publisher(BlobMotorPos, f"{p}/blob/motor_pos", 10)

    def reset_cache(self) -> None:
        self.cache = BlobUplinkCache()
        self._motor_ts = {MSG_AGV_MOTOR04: 0, MSG_AGV_MOTOR58: 0}

    def handle_frame(self, msg_id: int, blob_seq: int, payload: bytes, stamp) -> bool:
        """Parse one BLOB payload, update cache, publish typed topics. Returns uplink active."""
        updated_status = False
        updated_ext = False

        if msg_id == MSG_AGV_MOTION:
            motion = parse_agv_motion(payload)
            if motion is None:
                return False
            self.cache.motion = motion
            self.pub_motion.publish(motion_to_msg(motion, stamp, blob_seq))
            updated_status = True
            updated_ext = True
        elif msg_id == MSG_MCU_STATUS:
            mcu = parse_mcu_status(payload)
            if mcu is None:
                return False
            self.cache.mcu = mcu
            self.pub_mcu.publish(mcu_to_msg(mcu, stamp, blob_seq))
            updated_status = True
            updated_ext = True
        elif msg_id == MSG_SENSOR_BLOB:
            sensor = parse_sensor_blob(payload)
            if sensor is None:
                return False
            self.cache.sensor = sensor
            self.pub_sensor.publish(sensor_to_msg(sensor, stamp, blob_seq))
            updated_status = True
        elif msg_id == MSG_GPS_COMPACT:
            gps = parse_gps_compact(payload)
            if gps is None:
                return False
            self.pub_gps.publish(gps_to_msg(gps, stamp, blob_seq))
            self.pub_gps_a.publish(gps_compact_to_frame_a(gps, stamp))
            self.pub_gps_b.publish(gps_compact_to_frame_b(gps, stamp))
            self.pub_gps_c.publish(gps_compact_to_frame_c(gps, stamp))
        elif msg_id == MSG_AGV_MOTOR04:
            motors = parse_agv_motor04(payload)
            if motors is None:
                return False
            self.cache.update_motors(motors, base=0)
            ts = _u32_ts(payload)
            self._motor_ts[MSG_AGV_MOTOR04] = ts
            self.pub_motor04.publish(
                motor_group_to_msg(motors, ts, stamp, blob_seq, motor_base_index=0)
            )
            updated_ext = True
        elif msg_id == MSG_AGV_MOTOR58:
            motors = parse_agv_motor58(payload)
            if motors is None:
                return False
            self.cache.update_motors(motors, base=4)
            ts = _u32_ts(payload)
            self._motor_ts[MSG_AGV_MOTOR58] = ts
            self.pub_motor58.publish(
                motor_group_to_msg(motors, ts, stamp, blob_seq, motor_base_index=4)
            )
            updated_ext = True
        elif msg_id == MSG_AGV_ENERGY:
            energy = parse_agv_energy(payload)
            if energy is None:
                return False
            self.cache.energy = energy
            self.pub_energy.publish(energy_to_msg(energy, stamp, blob_seq))
            updated_status = True
        elif msg_id == MSG_AGV_MOTOR_POS:
            pos = parse_agv_motor_pos(payload)
            if pos is None:
                return False
            self.cache.motor_pos = pos
            self.pub_motor_pos.publish(motor_pos_to_msg(pos, stamp, blob_seq))
        else:
            return False

        if updated_status:
            status_msg = cache_to_v3_status(self.cache, stamp)
            if status_msg is not None:
                self.pub_v3_status.publish(status_msg)
        if updated_ext:
            ext_msg = cache_to_v3_ext(self.cache, stamp)
            if ext_msg is not None:
                self.pub_v3_ext.publish(ext_msg)

        return msg_id in self.UPLINK_WATCH_IDS

    def status_publish_count(self) -> int:
        return 1 if self.cache.motion or self.cache.mcu else 0


def _u32_ts(payload: bytes) -> int:
    if len(payload) < 4:
        return 0
    return int.from_bytes(payload[0:4], "big")
