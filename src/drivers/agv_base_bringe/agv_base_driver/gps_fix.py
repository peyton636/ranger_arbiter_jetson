"""MCU GPS 帧融合为 NavSatFix /fix（供 agv_base_eth_bringe 使用）。"""

from __future__ import annotations

import math
import time

from jetson_mcu_msgs.msg import GpsFrameA, GpsFrameB, GpsFrameC, TimeSyncResponse
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from agv_base_driver.blob_time_stamp import mcu_virtual_to_ros_stamp
from agv_base_driver.time_stamp import estimate_utc_sec, mono_ms, utc_sec_to_stamp

POS_VALID = 1 << 0
HDOP_INVALID = 65535
ALT_INVALID = 32767


class GpsFixHelper:
    def __init__(
        self,
        node: Node,
        *,
        fix_topic: str = "/fix",
        frame_id: str = "gps",
        max_set_age_ms: int = 500,
        use_time_sync_stamp: bool = True,
    ) -> None:
        self._node = node
        self._logger = node.get_logger()
        self._frame_id = frame_id
        self._max_set_age_s = max_set_age_ms / 1000.0
        self._use_time_sync_stamp = use_time_sync_stamp

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

        self._pub_fix = node.create_publisher(NavSatFix, fix_topic, 10)

    def on_time_sync(self, msg: TimeSyncResponse) -> None:
        if msg.offset_valid:
            self._offset_ms = msg.offset_ms
            self._time_sync_ready = True
        if msg.utc_unix_sec > 0:
            self._utc_anchor_sec = int(msg.utc_unix_sec)
            self._query_tick_ms = int(msg.system_tick_ms)
        elif msg.cmd_echo == 0 and msg.system_tick_ms > 0 and msg.utc_unix_sec == 0:
            self._query_tick_ms = int(msg.system_tick_ms)

    def on_gps_frames(self, a: GpsFrameA, b: GpsFrameB, c: GpsFrameC) -> None:
        now = time.monotonic()
        self._latest_a = a
        self._latest_b = b
        self._latest_c = c
        self._last_a_time = now
        self._last_b_time = now
        self._last_c_time = now
        self._last_gps_rx_time = now
        self.try_publish()

    def _set_fresh(self) -> bool:
        if self._latest_a is None or self._latest_b is None or self._latest_c is None:
            return False
        now = time.monotonic()
        t_max = max(self._last_a_time, self._last_b_time, self._last_c_time)
        t_min = min(self._last_a_time, self._last_b_time, self._last_c_time)
        return (now - t_max) <= self._max_set_age_s and (t_max - t_min) <= self._max_set_age_s

    def _stamp_for_observation(self):
        if not self._use_time_sync_stamp:
            return self._node.get_clock().now().to_msg()
        if not self._time_sync_ready:
            return self._node.get_clock().now().to_msg()
        now_ms = mono_ms()
        utc_sec = estimate_utc_sec(
            now_ms,
            self._offset_ms,
            self._utc_anchor_sec,
            self._query_tick_ms,
        )
        if utc_sec is not None:
            return utc_sec_to_stamp(utc_sec)
        return mcu_virtual_to_ros_stamp(self._offset_ms, True, self._node.get_clock())

    def try_publish(self) -> None:
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
            msg.altitude = c.alt_dm / 10.0 if c.alt_dm != ALT_INVALID else float("nan")
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
        msg.position_covariance[0] = hdop**2
        msg.position_covariance[4] = hdop**2
        msg.position_covariance[8] = (2.0 * hdop) ** 2
        self._pub_fix.publish(msg)

        now = time.monotonic()
        if now - self._last_log_time >= 10.0:
            self._last_log_time = now
            if pos_valid:
                self._logger.info(
                    f"GPS fix: lat={msg.latitude:.7f} lon={msg.longitude:.7f} sv={a.num_sv}",
                    throttle_duration_sec=10.0,
                )
