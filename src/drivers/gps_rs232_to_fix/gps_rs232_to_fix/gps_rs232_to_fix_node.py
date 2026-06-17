#!/usr/bin/env python3
"""Merge /jetson_rs232/gps/{a,b,c} into sensor_msgs/NavSatFix /fix."""

from __future__ import annotations

import math
import time

import rclpy
from jetson_can_msgs.msg import GpsFrameA, GpsFrameB, GpsFrameC, TimeSyncResponse
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from gps_rs232_to_fix.time_stamp import estimate_utc_sec, mono_ms, utc_sec_to_stamp

POS_VALID = 1 << 0
HDOP_INVALID = 65535
ALT_INVALID = 32767
CMD_PING = 0x03
CMD_START = 0x02


class GpsRs232ToFixNode(Node):
    def __init__(self) -> None:
        super().__init__("gps_rs232_to_fix")

        self.declare_parameter("link_type", "rs232")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("max_set_age_ms", 500)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("use_time_sync_stamp", True)
        self.declare_parameter("time_sync_topic", "")

        link_type = self.get_parameter("link_type").get_parameter_value().string_value
        prefix = f"/jetson_{link_type}"

        self._max_set_age_s = (
            self.get_parameter("max_set_age_ms").get_parameter_value().integer_value
            / 1000.0
        )
        self._frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        fix_topic = (
            self.get_parameter("fix_topic").get_parameter_value().string_value
        )
        rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self._use_time_sync_stamp = (
            self.get_parameter("use_time_sync_stamp").get_parameter_value().bool_value
        )
        time_sync_topic = (
            self.get_parameter("time_sync_topic").get_parameter_value().string_value
        )
        if not time_sync_topic:
            time_sync_topic = f"{prefix}/time_sync"

        self._latest_a: GpsFrameA | None = None
        self._latest_b: GpsFrameB | None = None
        self._latest_c: GpsFrameC | None = None
        self._last_a_time = 0.0
        self._last_b_time = 0.0
        self._last_c_time = 0.0
        self._last_log_time = 0.0
        self._had_fix = False
        self._last_gps_rx_time = 0.0

        self._offset_ms = 0.0
        self._utc_anchor_sec = 0
        self._query_tick_ms = 0
        self._time_sync_ready = False

        self._pub_fix = self.create_publisher(NavSatFix, fix_topic, 10)
        self.create_subscription(GpsFrameA, f"{prefix}/gps/a", self._a_cb, 10)
        self.create_subscription(GpsFrameB, f"{prefix}/gps/b", self._b_cb, 10)
        self.create_subscription(GpsFrameC, f"{prefix}/gps/c", self._c_cb, 10)
        if self._use_time_sync_stamp:
            self.create_subscription(
                TimeSyncResponse, time_sync_topic, self._time_sync_cb, 10
            )

        if rate_hz > 0:
            self.create_timer(1.0 / rate_hz, self._try_publish)
        self.create_timer(10.0, self._check_gps_stale)

        stamp_mode = "time_sync+UTC锚点" if self._use_time_sync_stamp else "ROS时钟"
        self.get_logger().info(
            f"gps_rs232_to_fix: {prefix}/gps/{{a,b,c}} → {fix_topic} "
            f"stamp={stamp_mode} sync={time_sync_topic}"
        )

    def _time_sync_cb(self, msg: TimeSyncResponse) -> None:
        if msg.cmd_echo in (CMD_PING, CMD_START) or msg.rtt_ms > 0:
            self._offset_ms = msg.offset_ms
            self._time_sync_ready = True
        if msg.utc_unix_sec > 0:
            self._utc_anchor_sec = int(msg.utc_unix_sec)
            self._query_tick_ms = int(msg.system_tick_ms)
            self.get_logger().info(
                f"TimeSync QUERY 锚点: utc={self._utc_anchor_sec} "
                f"mcu_tick={self._query_tick_ms}",
                throttle_duration_sec=30.0,
            )
        elif msg.cmd_echo == 0 and msg.system_tick_ms > 0 and msg.utc_unix_sec == 0:
            # QUERY 无 GPS：仍更新 tick 锚点，UTC 保持 0
            self._query_tick_ms = int(msg.system_tick_ms)

    def _a_cb(self, msg: GpsFrameA) -> None:
        self._latest_a = msg
        self._last_a_time = time.monotonic()
        self._last_gps_rx_time = self._last_a_time

    def _b_cb(self, msg: GpsFrameB) -> None:
        self._latest_b = msg
        self._last_b_time = time.monotonic()
        self._last_gps_rx_time = self._last_b_time

    def _c_cb(self, msg: GpsFrameC) -> None:
        self._latest_c = msg
        self._last_c_time = time.monotonic()
        self._last_gps_rx_time = self._last_c_time
        self._try_publish()

    def _check_gps_stale(self) -> None:
        if self._last_gps_rx_time <= 0:
            self.get_logger().warn(
                "尚未收到 /jetson_rs232/gps/*，请确认 rs232_gateway 在跑且 F407 有 GPS fix",
                throttle_duration_sec=10.0,
            )

    def _stamp_for_observation(self) -> object:
        if not self._use_time_sync_stamp or not self._time_sync_ready:
            return self.get_clock().now().to_msg()

        now_ms = mono_ms()
        utc_sec = estimate_utc_sec(
            now_ms,
            self._offset_ms,
            self._utc_anchor_sec,
            self._query_tick_ms,
        )
        if utc_sec is not None:
            return utc_sec_to_stamp(utc_sec)
        return self.get_clock().now().to_msg()

    def _set_fresh(self) -> bool:
        if self._latest_a is None or self._latest_b is None or self._latest_c is None:
            return False
        now = time.monotonic()
        t_max = max(self._last_a_time, self._last_b_time, self._last_c_time)
        t_min = min(self._last_a_time, self._last_b_time, self._last_c_time)
        return (now - t_max) <= self._max_set_age_s and (t_max - t_min) <= self._max_set_age_s

    def _try_publish(self) -> None:
        if not self._set_fresh():
            return

        a = self._latest_a
        b = self._latest_b
        c = self._latest_c
        assert a is not None and b is not None and c is not None

        msg = NavSatFix()
        msg.header.stamp = self._stamp_for_observation()
        msg.header.frame_id = self._frame_id
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        pos_valid = bool(a.flags & POS_VALID)
        if pos_valid:
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.latitude = b.lat_e7 / 1e7
            msg.longitude = c.lon_e7 / 1e7
            if c.alt_dm != ALT_INVALID:
                msg.altitude = c.alt_dm / 10.0
            else:
                msg.altitude = float("nan")
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = float("nan")
            msg.longitude = float("nan")
            msg.altitude = float("nan")
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        if a.hdop_x100 != HDOP_INVALID and a.hdop_x100 > 0:
            hdop = a.hdop_x100 / 100.0
        elif pos_valid:
            hdop = 4.0
        else:
            hdop = 9999.0

        msg.position_covariance[0] = hdop ** 2
        msg.position_covariance[4] = hdop ** 2
        msg.position_covariance[8] = (2.0 * hdop) ** 2

        self._pub_fix.publish(msg)
        self._log_fix(pos_valid, a.num_sv, msg.latitude, msg.longitude, msg.header.stamp.sec)

    def _log_fix(
        self,
        pos_valid: bool,
        num_sv: int,
        lat: float,
        lon: float,
        stamp_sec: int,
    ) -> None:
        now = time.monotonic()
        if now - self._last_log_time < 10.0:
            return
        self._last_log_time = now
        utc_note = ""
        if self._utc_anchor_sec > 0:
            utc_note = f", stamp_sec={stamp_sec}(UTC锚点)"
        elif self._use_time_sync_stamp and self._time_sync_ready:
            utc_note = ", stamp=ROS时钟(无UTC锚点)"

        if pos_valid and not math.isnan(lat) and not math.isnan(lon):
            if not self._had_fix:
                self.get_logger().info(
                    f"GPS fix 有效: lat={lat:.7f}, lon={lon:.7f}, 卫星={num_sv}{utc_note}"
                )
                self._had_fix = True
        else:
            self.get_logger().info(
                f"GPS 无 fix（flags 无 POS_VALID）: 卫星={num_sv}，仍发布 /fix status=NO_FIX",
                throttle_duration_sec=10.0,
            )
            self._had_fix = False


def main() -> None:
    rclpy.init()
    node = GpsRs232ToFixNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
