"""MCU ????? UDP ??��??BLOB v2 + TimeSync?????? agv_base_driver ??????????"""

from __future__ import annotations

import queue
import threading
import time
from typing import Callable

from jetson_mcu_msgs.msg import BlobSensorCfg, TimeSyncResponse, V3Command, V3ExtStatus, V3Status
from jetson_mcu_msgs.msg import GpsFrameA, GpsFrameB, GpsFrameC
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Bool

from agv_base_driver.blob_codec import encode_agv_control, encode_sensor_cfg, v3_command_to_blob
from agv_base_driver.blob_time_stamp import time_sync_response_ros_stamp
from agv_base_driver.blob_topic_pub import BlobTopicPublisher
from agv_base_driver.blob_udp import EthBlobParser, EthRxItem, EthRxStats
from agv_base_driver.jetson_protocol import UPLINK_TIMEOUT_MS
from agv_base_driver.service_codec import payload_to_time_sync
from agv_base_driver.service_frame import CAN_ID_TIME_SYNC_RSP
from agv_base_driver.time_sync import CMD_START, JetsonTimeSync, mono_ms
from agv_base_driver.udp_link import UdpLink

_PARAM_FLOAT = ParameterDescriptor(dynamic_typing=True)

OnV3Status = Callable[[V3Status], None]
OnV3Ext = Callable[[V3ExtStatus], None]
OnGpsFrames = Callable[[GpsFrameA, GpsFrameB, GpsFrameC], None]
OnTimeSync = Callable[[TimeSyncResponse], None]


def _param_float(node: Node, name: str) -> float:
    return float(node.get_parameter(name).value)


class McuEthBridge:
    """UDP ? BLOB/TimeSync????????????????? /command ?? Topic ?????"""

    def __init__(
        self,
        node: Node,
        topic_prefix: str = "/jetson_eth",
        *,
        enable_ros_uplink_publish: bool = True,
        enable_ros_link_publish: bool | None = None,
        enable_ros_time_sync_publish: bool | None = None,
        enable_sensor_cfg_sub: bool | None = None,
        on_v3_status: OnV3Status | None = None,
        on_v3_ext: OnV3Ext | None = None,
        on_gps_frames: OnGpsFrames | None = None,
        on_time_sync: OnTimeSync | None = None,
    ) -> None:
        self._node = node
        self._logger = node.get_logger()
        self._prefix = topic_prefix.rstrip("/")

        bind_ip = node.get_parameter("bind_ip").get_parameter_value().string_value
        bind_device = node.get_parameter("bind_device").get_parameter_value().string_value
        local_port = node.get_parameter("local_port").get_parameter_value().integer_value
        mcu_ip = node.get_parameter("mcu_ip").get_parameter_value().string_value
        mcu_port = node.get_parameter("mcu_port").get_parameter_value().integer_value
        self._tx_rate_hz = _param_float(node, "tx_rate_hz")
        self._uplink_timeout_s = (
            node.get_parameter("uplink_timeout_ms").get_parameter_value().integer_value
            / 1000.0
        )
        self._link_down_timeout_s = (
            node.get_parameter("link_down_timeout_ms").get_parameter_value().integer_value
            / 1000.0
        )
        self._heartbeat_mode_req = (
            node.get_parameter("heartbeat_mode_req").get_parameter_value().integer_value
        )
        self._debug_rx_stats_interval_s = _param_float(node, "debug_rx_stats_interval_s")
        rx_dispatch_hz = _param_float(node, "rx_dispatch_hz")
        self._time_sync_enable = (
            node.get_parameter("time_sync_enable").get_parameter_value().bool_value
        )
        self._time_sync = JetsonTimeSync(
            session_id=node.get_parameter("time_sync_session_id").get_parameter_value().integer_value,
            ping_burst_count=node.get_parameter("time_sync_ping_burst").get_parameter_value().integer_value,
            ping_burst_interval_s=_param_float(node, "time_sync_ping_burst_interval_s"),
            ping_interval_s=_param_float(node, "time_sync_ping_interval_s"),
            query_interval_s=_param_float(node, "time_sync_query_interval_s"),
            rtt_warn_ms=_param_float(node, "time_sync_rtt_warn_ms"),
        )

        self._link = UdpLink(
            bind_ip, local_port, mcu_ip, mcu_port,
            bind_device=bind_device,
            logger=self._logger.info,
        )
        self._parser = EthBlobParser()
        self._tx_seq = 0
        self._cfg_tx_seq = 0
        self._last_blob_uplink_time = 0.0
        self._latest_command: V3Command | None = None
        self._enable_ros_uplink = enable_ros_uplink_publish
        pub_link = (
            enable_ros_link_publish
            if enable_ros_link_publish is not None
            else enable_ros_uplink_publish
        )
        pub_ts = (
            enable_ros_time_sync_publish
            if enable_ros_time_sync_publish is not None
            else enable_ros_uplink_publish
        )
        sub_cfg = (
            enable_sensor_cfg_sub
            if enable_sensor_cfg_sub is not None
            else enable_ros_uplink_publish
        )
        self._blob_pub = BlobTopicPublisher(
            node, self._prefix, enable_ros_publish=enable_ros_uplink_publish
        )
        self._blob_pub.set_uplink_callbacks(
            on_v3_status=on_v3_status,
            on_v3_ext=on_v3_ext,
            on_gps_frames=on_gps_frames,
        )
        self._on_time_sync = on_time_sync
        self._pub_status_interval = 0
        self._rx_queue: queue.Queue[EthRxItem] = queue.Queue(maxsize=4096)
        self._rx_stop = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._last_stats_log = time.monotonic()
        self._link_was_up = False
        self._link_uplink_up = False
        self._shutting_down = False
        self._reconnect_interval_s = _param_float(node, "reconnect_interval_s")
        self._next_reconnect = 0.0

        self._pub_link = None
        self._pub_time_sync = None
        if pub_link:
            self._pub_link = node.create_publisher(Bool, f"{self._prefix}/link", 10)
        if pub_ts:
            self._pub_time_sync = node.create_publisher(
                TimeSyncResponse, f"{self._prefix}/time_sync", 10
            )
        if sub_cfg:
            node.create_subscription(
                BlobSensorCfg,
                f"{self._prefix}/blob/sensor_cfg",
                self._sensor_cfg_cb,
                10,
            )

        period = 1.0 / self._tx_rate_hz if self._tx_rate_hz > 0 else 0.02
        self._timer = node.create_timer(period, self._tick)
        rx_period = 1.0 / rx_dispatch_hz if rx_dispatch_hz > 0 else 0.02
        self._rx_dispatch_timer = node.create_timer(rx_period, self._dispatch_rx_only)

        if not self._ensure_udp():
            self._logger.warn("UDP not bound yet, will retry in timer")

        self._logger.info(
            f"MCU eth bind {bind_ip}:{local_port} -> {mcu_ip}:{mcu_port}, "
            f"{self._tx_rate_hz:.0f}Hz BLOB, time_sync="
            f"{'on' if self._time_sync_enable else 'off'}"
        )

    @staticmethod
    def declare_parameters(node: Node) -> None:
        node.declare_parameter("bind_ip", "192.168.10.201")
        node.declare_parameter("bind_device", "enp1s0f1")
        node.declare_parameter("local_port", 50002)
        node.declare_parameter("mcu_ip", "192.168.10.30")
        node.declare_parameter("mcu_port", 50001)
        node.declare_parameter("tx_rate_hz", 50.0, _PARAM_FLOAT)
        node.declare_parameter("uplink_timeout_ms", UPLINK_TIMEOUT_MS)
        node.declare_parameter("link_down_timeout_ms", 1500)
        node.declare_parameter("heartbeat_mode_req", 1)
        node.declare_parameter("debug_rx_stats_interval_s", 5.0, _PARAM_FLOAT)
        node.declare_parameter("rx_dispatch_hz", 50.0, _PARAM_FLOAT)
        node.declare_parameter("time_sync_enable", True)
        node.declare_parameter("time_sync_session_id", 1)
        node.declare_parameter("time_sync_ping_burst", 10)
        node.declare_parameter("time_sync_ping_burst_interval_s", 0.1, _PARAM_FLOAT)
        node.declare_parameter("time_sync_ping_interval_s", 1.0, _PARAM_FLOAT)
        node.declare_parameter("time_sync_query_interval_s", 10.0, _PARAM_FLOAT)
        node.declare_parameter("time_sync_rtt_warn_ms", 50.0, _PARAM_FLOAT)
        node.declare_parameter("reconnect_interval_s", 1.0, _PARAM_FLOAT)
        node.declare_parameter("ros_uplink_publish", True)
        node.declare_parameter("ros_link_publish", True)
        node.declare_parameter("ros_time_sync_publish", True)
        node.declare_parameter("ros_sensor_cfg_sub", True)

    @staticmethod
    def read_ros_publish_flags(node: Node) -> dict:
        return {
            "enable_ros_uplink_publish": node.get_parameter(
                "ros_uplink_publish"
            ).get_parameter_value().bool_value,
            "enable_ros_link_publish": node.get_parameter(
                "ros_link_publish"
            ).get_parameter_value().bool_value,
            "enable_ros_time_sync_publish": node.get_parameter(
                "ros_time_sync_publish"
            ).get_parameter_value().bool_value,
            "enable_sensor_cfg_sub": node.get_parameter(
                "ros_sensor_cfg_sub"
            ).get_parameter_value().bool_value,
        }

    @property
    def blob_pub(self) -> BlobTopicPublisher:
        return self._blob_pub

    @property
    def link_connected(self) -> bool:
        return self._link.connected

    @property
    def link_uplink_up(self) -> bool:
        """True when BLOB uplink recently alive (hysteresis, avoids GUI flapping)."""
        return self._link_uplink_up

    def set_command(self, cmd: V3Command | None) -> None:
        self._latest_command = cmd

    def _sensor_cfg_cb(self, msg: BlobSensorCfg) -> None:
        if not self._link.connected:
            return
        ts = msg.timestamp_ms if msg.timestamp_ms else self._blob_pub.mcu_tx_tick_ms()
        try:
            frame = encode_sensor_cfg(
                self._cfg_tx_seq, ts,
                threshold_mm=msg.threshold_mm,
                enable_mask=msg.enable_mask,
            )
            self._link.send(frame)
            self._cfg_tx_seq = (self._cfg_tx_seq + 1) & 0xFF
        except OSError as exc:
            self._logger.warn(f"sensor_cfg send failed: {exc}")

    def _send_service_frames(self, frames: list[bytes]) -> None:
        for frame in frames:
            self._link.send(frame)

    def _ensure_udp(self) -> bool:
        if self._link.connected:
            return True
        now = time.monotonic()
        if now < self._next_reconnect:
            return False
        self._next_reconnect = now + self._reconnect_interval_s
        if self._link.open():
            self._parser = EthBlobParser()
            self._blob_pub.reset_cache()
            self._last_blob_uplink_time = 0.0
            self._last_stats_log = time.monotonic()
            self._start_rx_thread()
            self._prime_downlink()
            self._logger.info("UDP reconnected")
            return True
        return False

    def _start_rx_thread(self) -> None:
        self._stop_rx_thread()
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="eth_udp_rx", daemon=True
        )
        self._rx_thread.start()

    def _stop_rx_thread(self) -> None:
        self._rx_stop.set()
        if self._rx_thread is not None and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        self._rx_thread = None
        if not self._shutting_down:
            self._process_rx_queue()
        while True:
            try:
                self._rx_queue.get_nowait()
            except queue.Empty:
                break

    def _rx_loop(self) -> None:
        while not self._rx_stop.is_set():
            if not self._link.connected:
                time.sleep(0.01)
                continue
            for datagram in self._link.recv_available():
                parsed = self._parser.parse_datagram(datagram)
                if parsed is None:
                    continue
                if parsed[0] == "svc":
                    _, can_id, payload = parsed
                    parsed = ("svc", can_id, payload, mono_ms())
                try:
                    self._rx_queue.put(parsed, timeout=0.05)
                except queue.Full:
                    self._logger.warn(
                        "RX queue full, drop one UDP frame",
                        throttle_duration_sec=5.0,
                    )
            time.sleep(0.002)

    def _prime_downlink(self, frames: int = 10) -> None:
        if not self._link.connected:
            return
        for _ in range(frames):
            try:
                frame = encode_agv_control(
                    self._tx_seq, self._blob_pub.mcu_tx_tick_ms(),
                    control_mode=self._heartbeat_mode_req,
                )
                self._link.send(frame)
                self._tx_seq = (self._tx_seq + 1) & 0xFF
            except OSError:
                self._link.close()
                break
            time.sleep(0.02)
        if self._time_sync_enable:
            try:
                self._link.send(self._time_sync.begin_session())
                self._logger.info(
                    f"TimeSync START session={self._time_sync.session_id}, "
                    f"burst={self._time_sync.ping_burst_count}"
                )
            except OSError:
                self._link.close()

    def _publish_link(self, up: bool) -> None:
        if self._pub_link is None:
            if up != self._link_was_up:
                self._logger.info("link: up" if up else "link: down")
                self._link_was_up = up
            return
        msg = Bool()
        msg.data = up
        self._pub_link.publish(msg)
        if up != self._link_was_up:
            self._logger.info("link: up" if up else "link: down")
            self._link_was_up = up

    def _dispatch_rx_only(self) -> None:
        self._process_rx_queue()

    def _tick(self) -> None:
        connected = self._ensure_udp()
        self._publish_link(connected)
        self._process_rx_queue()
        if not connected:
            return
        try:
            frame = v3_command_to_blob(
                self._tx_seq, self._latest_command, self._blob_pub.mcu_tx_tick_ms()
            )
            self._link.send(frame)
            self._tx_seq = (self._tx_seq + 1) & 0xFF
            if self._time_sync_enable:
                self._send_service_frames(self._time_sync.tick())
        except OSError as exc:
            self._logger.warn(f"UDP send failed: {exc}")
            self._link.close()
            self._stop_rx_thread()
            return
        self._maybe_log_rx_stats()
        self._update_link_uplink()

    def _update_link_uplink(self) -> None:
        if not self._link.connected:
            if self._link_uplink_up:
                self._link_uplink_up = False
                self._logger.info("link uplink: down (UDP closed)")
            return
        if self._last_blob_uplink_time <= 0:
            return
        age_s = time.monotonic() - self._last_blob_uplink_time
        if self._link_uplink_up:
            if age_s > self._link_down_timeout_s:
                self._link_uplink_up = False
                self._logger.warn(
                    f"link uplink: down ({age_s * 1000:.0f}ms no BLOB uplink)",
                    throttle_duration_sec=5.0,
                )
        elif age_s <= self._uplink_timeout_s:
            self._link_uplink_up = True
            self._logger.info("link uplink: up (BLOB receiving)")

    def _process_rx_queue(self) -> None:
        while True:
            try:
                item = self._rx_queue.get_nowait()
            except queue.Empty:
                break
            if item[0] == "blob":
                _, msg_id, blob_seq, payload = item
                if self._blob_pub.handle_frame(msg_id, blob_seq, payload):
                    self._last_blob_uplink_time = time.monotonic()
                    self._pub_status_interval += 1
            else:
                _, can_id, payload, t4_ms = item
                if can_id == CAN_ID_TIME_SYNC_RSP:
                    self._handle_time_sync(payload, t4_ms=t4_ms)

    def _handle_time_sync(self, payload: bytes, t4_ms: float) -> None:
        update = None
        if self._time_sync_enable:
            update = self._time_sync.on_response(payload, t4_ms=t4_ms)
            if update is not None:
                if update.format == "ping":
                    if update.cmd_echo == CMD_START:
                        self._logger.info(
                            f"TimeSync START rtt={update.rtt_ms:.2f}ms "
                            f"offset={update.offset_ms:.2f}ms session={update.seq_echo}",
                            throttle_duration_sec=5.0,
                        )
                    else:
                        self._logger.info(
                            f"TimeSync PING rtt={update.rtt_ms:.2f}ms "
                            f"offset={update.offset_ms:.2f}ms seq={update.seq_echo}",
                            throttle_duration_sec=2.0,
                        )
                elif update.format == "query":
                    self._logger.info(
                        f"TimeSync QUERY tick={update.system_tick_ms} "
                        f"utc={update.utc_unix_sec}",
                        throttle_duration_sec=10.0,
                    )
        if update is not None and update.offset_valid:
            self._blob_pub.update_time_sync(update.offset_ms, True)
        mcu_tick = 0
        if update is not None:
            if update.format == "ping" and update.mcu_tick_rx:
                mcu_tick = int(update.mcu_tick_rx)
            elif update.system_tick_ms:
                mcu_tick = int(update.system_tick_ms)
        stamp = time_sync_response_ros_stamp(
            self._node.get_clock(),
            self._time_sync.offset_ms,
            self._time_sync.offset_valid,
            mcu_tick,
        )
        ts_msg = payload_to_time_sync(payload, stamp, update=update)
        if self._pub_time_sync is not None:
            self._pub_time_sync.publish(ts_msg)
        if self._on_time_sync is not None:
            self._on_time_sync(ts_msg)

    def _maybe_log_rx_stats(self) -> None:
        if self._debug_rx_stats_interval_s <= 0:
            return
        now = time.monotonic()
        if now - self._last_stats_log < self._debug_rx_stats_interval_s:
            return
        self._last_stats_log = now
        s = self._parser.stats.snapshot()
        self._logger.info(
            "RX +{:.0f}s: bytes={} pkts={} bad={} blob02={} blob03={} blob04={} "
            "blob05={} blob_other={} svc={} pub_status={}".format(
                self._debug_rx_stats_interval_s,
                s["bytes_in"], s["pkts_in"], s["bad_pkts"],
                s["blob_02"], s["blob_03"], s["blob_04"], s["blob_05"],
                s["blob_other"], s["svc_rx"], self._pub_status_interval,
            )
        )
        self._pub_status_interval = 0
        self._parser.stats = EthRxStats()

    def shutdown(self) -> None:
        self._shutting_down = True
        self._stop_rx_thread()
        if self._time_sync_enable and self._link.connected:
            try:
                self._link.send(self._time_sync.stop_session())
                self._logger.info("TimeSync STOP sent")
            except OSError:
                pass
        self._link.close()
