#!/usr/bin/env python3
"""Ethernet UDP gateway: BLOB v2 + 0xA5 service frames <-> /jetson_eth/* topics."""

from __future__ import annotations

import queue
import threading
import time

import rclpy
from jetson_can_msgs.msg import BlobSensorCfg, TimeSyncResponse, V3Command
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Bool

from ds_jetson_bridge.jetson_protocol import UPLINK_TIMEOUT_MS

from eth_gateway.blob_udp import EthBlobParser, EthRxItem, EthRxStats
from eth_gateway.udp_link import UdpLink
from rs232_gateway.blob_codec import encode_agv_control, encode_sensor_cfg, v3_command_to_blob
from rs232_gateway.blob_topic_pub import BlobTopicPublisher
from rs232_gateway.service_codec import payload_to_time_sync
from rs232_gateway.service_frame import CAN_ID_TIME_SYNC_RSP
from rs232_gateway.time_sync import CMD_START, JetsonTimeSync, mono_ms

_PARAM_FLOAT = ParameterDescriptor(dynamic_typing=True)
_TOPIC_PREFIX = "/jetson_eth"


def _param_float(node: Node, name: str) -> float:
    return float(node.get_parameter(name).value)


class EthGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("eth_gateway")

        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("local_port", 50002)
        self.declare_parameter("mcu_ip", "192.168.10.30")
        self.declare_parameter("mcu_port", 50001)
        self.declare_parameter("tx_rate_hz", 50.0, _PARAM_FLOAT)
        self.declare_parameter("uplink_timeout_ms", UPLINK_TIMEOUT_MS)
        self.declare_parameter("heartbeat_mode_req", 1)
        self.declare_parameter("debug_rx_stats_interval_s", 5.0, _PARAM_FLOAT)
        self.declare_parameter("rx_dispatch_hz", 50.0, _PARAM_FLOAT)
        self.declare_parameter("time_sync_enable", True)
        self.declare_parameter("time_sync_session_id", 1)
        self.declare_parameter("time_sync_ping_burst", 10)
        self.declare_parameter("time_sync_ping_burst_interval_s", 0.1, _PARAM_FLOAT)
        self.declare_parameter("time_sync_ping_interval_s", 1.0, _PARAM_FLOAT)
        self.declare_parameter("time_sync_query_interval_s", 10.0, _PARAM_FLOAT)
        self.declare_parameter("time_sync_rtt_warn_ms", 50.0, _PARAM_FLOAT)

        bind_ip = self.get_parameter("bind_ip").get_parameter_value().string_value
        local_port = self.get_parameter("local_port").get_parameter_value().integer_value
        mcu_ip = self.get_parameter("mcu_ip").get_parameter_value().string_value
        mcu_port = self.get_parameter("mcu_port").get_parameter_value().integer_value
        self._tx_rate_hz = _param_float(self, "tx_rate_hz")
        self._uplink_timeout_s = (
            self.get_parameter("uplink_timeout_ms").get_parameter_value().integer_value
            / 1000.0
        )
        self._heartbeat_mode_req = (
            self.get_parameter("heartbeat_mode_req").get_parameter_value().integer_value
        )
        self._debug_rx_stats_interval_s = _param_float(self, "debug_rx_stats_interval_s")
        rx_dispatch_hz = _param_float(self, "rx_dispatch_hz")
        self._time_sync_enable = (
            self.get_parameter("time_sync_enable").get_parameter_value().bool_value
        )
        self._time_sync = JetsonTimeSync(
            session_id=self.get_parameter("time_sync_session_id").get_parameter_value().integer_value,
            ping_burst_count=self.get_parameter("time_sync_ping_burst").get_parameter_value().integer_value,
            ping_burst_interval_s=_param_float(self, "time_sync_ping_burst_interval_s"),
            ping_interval_s=_param_float(self, "time_sync_ping_interval_s"),
            query_interval_s=_param_float(self, "time_sync_query_interval_s"),
            rtt_warn_ms=_param_float(self, "time_sync_rtt_warn_ms"),
        )

        self._link = UdpLink(
            bind_ip,
            local_port,
            mcu_ip,
            mcu_port,
            logger=self.get_logger().info,
        )
        self._parser = EthBlobParser()
        self._tx_seq = 0
        self._cfg_tx_seq = 0
        self._last_blob_uplink_time = 0.0
        self._latest_command: V3Command | None = None
        self._blob_pub = BlobTopicPublisher(self, _TOPIC_PREFIX)
        self._pub_status_interval = 0
        self._rx_queue: queue.Queue[EthRxItem] = queue.Queue(maxsize=4096)
        self._rx_stop = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._last_stats_log = time.monotonic()
        self._link_was_up = False
        self._shutting_down = False

        self._pub_link = self.create_publisher(Bool, f"{_TOPIC_PREFIX}/link", 10)
        self._pub_time_sync = self.create_publisher(
            TimeSyncResponse, f"{_TOPIC_PREFIX}/time_sync", 10
        )
        self._sub_command = self.create_subscription(
            V3Command, f"{_TOPIC_PREFIX}/command", self._command_cb, 10
        )
        self._sub_sensor_cfg = self.create_subscription(
            BlobSensorCfg,
            f"{_TOPIC_PREFIX}/blob/sensor_cfg",
            self._sensor_cfg_cb,
            10,
        )

        period = 1.0 / self._tx_rate_hz if self._tx_rate_hz > 0 else 0.02
        self._timer = self.create_timer(period, self._tick)
        rx_period = 1.0 / rx_dispatch_hz if rx_dispatch_hz > 0 else 0.02
        self._rx_dispatch_timer = self.create_timer(rx_period, self._dispatch_rx_only)

        if not self._ensure_udp():
            self.get_logger().warn("UDP not bound yet, will retry in timer")

        self.get_logger().info(
            f"eth_gateway bind {bind_ip}:{local_port} -> MCU {mcu_ip}:{mcu_port}, "
            f"{self._tx_rate_hz:.0f}Hz BLOB v2, time_sync="
            f"{'on' if self._time_sync_enable else 'off'}, "
            f"topics {_TOPIC_PREFIX}/{{v3_*,blob/*,command,link,time_sync}}"
        )

    def _command_cb(self, msg: V3Command) -> None:
        self._latest_command = msg

    def _sensor_cfg_cb(self, msg: BlobSensorCfg) -> None:
        if not self._link.connected:
            return
        ts = msg.timestamp_ms if msg.timestamp_ms else int(mono_ms())
        try:
            frame = encode_sensor_cfg(
                self._cfg_tx_seq,
                ts,
                threshold_mm=msg.threshold_mm,
                enable_mask=msg.enable_mask,
            )
            self._link.send(frame)
            self._cfg_tx_seq = (self._cfg_tx_seq + 1) & 0xFF
        except OSError as exc:
            self.get_logger().warn(f"sensor_cfg send failed: {exc}")

    def _send_service_frames(self, frames: list[bytes]) -> None:
        for frame in frames:
            self._link.send(frame)

    def _ensure_udp(self) -> bool:
        if self._link.connected:
            return True
        if self._link.open():
            self._parser = EthBlobParser()
            self._blob_pub.reset_cache()
            self._last_blob_uplink_time = 0.0
            self._last_stats_log = time.monotonic()
            self._start_rx_thread()
            self._prime_downlink()
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
                try:
                    self._rx_queue.put(parsed, timeout=0.05)
                except queue.Full:
                    self.get_logger().warn(
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
                    self._tx_seq, int(mono_ms()), control_mode=self._heartbeat_mode_req
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
                self.get_logger().info(
                    f"TimeSync START session={self._time_sync.session_id}, "
                    f"burst={self._time_sync.ping_burst_count}"
                )
            except OSError:
                self._link.close()

    def _publish_link(self, up: bool) -> None:
        msg = Bool()
        msg.data = up
        self._pub_link.publish(msg)
        if up != self._link_was_up:
            self.get_logger().info("link: up" if up else "link: down")
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
                self._tx_seq, self._latest_command, int(mono_ms())
            )
            self._link.send(frame)
            self._tx_seq = (self._tx_seq + 1) & 0xFF
            if self._time_sync_enable:
                self._send_service_frames(self._time_sync.tick())
        except OSError as exc:
            self.get_logger().warn(f"UDP send failed: {exc}")
            self._link.close()
            self._stop_rx_thread()
            return

        self._maybe_log_rx_stats()
        self._check_uplink_stale()

    def _process_rx_queue(self) -> None:
        stamp = self.get_clock().now().to_msg()
        while True:
            try:
                item = self._rx_queue.get_nowait()
            except queue.Empty:
                break
            if item[0] == "blob":
                _, msg_id, blob_seq, payload = item
                if self._blob_pub.handle_frame(msg_id, blob_seq, payload, stamp):
                    self._last_blob_uplink_time = time.monotonic()
                    self._pub_status_interval += 1
            else:
                _, can_id, payload = item
                if can_id == CAN_ID_TIME_SYNC_RSP:
                    self._handle_time_sync(payload, stamp, t4_ms=mono_ms())

    def _handle_time_sync(self, payload: bytes, stamp, t4_ms: float) -> None:
        update = None
        if self._time_sync_enable:
            update = self._time_sync.on_response(payload, t4_ms=t4_ms)
            if update is not None:
                if update.format == "ping":
                    if update.cmd_echo == CMD_START:
                        self.get_logger().info(
                            f"TimeSync START rtt={update.rtt_ms:.2f}ms "
                            f"offset={update.offset_ms:.2f}ms session={update.seq_echo}",
                            throttle_duration_sec=5.0,
                        )
                    else:
                        self.get_logger().info(
                            f"TimeSync PING rtt={update.rtt_ms:.2f}ms "
                            f"offset={update.offset_ms:.2f}ms "
                            f"mcu_tick={update.mcu_tick_rx} "
                            f"proc={update.proc_ms:.2f}ms "
                            f"seq={update.seq_echo}",
                            throttle_duration_sec=2.0,
                        )
                    if update.rtt_warn:
                        self.get_logger().warn(
                            f"TimeSync RTT ????: {update.rtt_ms:.2f}ms "
                            f"(???>{self._time_sync.rtt_warn_ms:.0f}ms)",
                            throttle_duration_sec=5.0,
                        )
                elif update.format == "query":
                    self.get_logger().info(
                        f"TimeSync QUERY tick={update.system_tick_ms} "
                        f"utc={update.utc_unix_sec}",
                        throttle_duration_sec=10.0,
                    )
        self._pub_time_sync.publish(
            payload_to_time_sync(payload, stamp, update=update)
        )

    def _maybe_log_rx_stats(self) -> None:
        if self._debug_rx_stats_interval_s <= 0:
            return
        now = time.monotonic()
        if now - self._last_stats_log < self._debug_rx_stats_interval_s:
            return
        self._last_stats_log = now
        s = self._parser.stats.snapshot()
        self.get_logger().info(
            "RX +{:.0f}s: bytes={} pkts={} bad={} blob02={} blob03={} blob04={} "
            "blob05={} blob_other={} svc={} pub_status={}".format(
                self._debug_rx_stats_interval_s,
                s["bytes_in"],
                s["pkts_in"],
                s["bad_pkts"],
                s["blob_02"],
                s["blob_03"],
                s["blob_04"],
                s["blob_05"],
                s["blob_other"],
                s["svc_rx"],
                self._pub_status_interval,
            )
        )
        self._pub_status_interval = 0
        self._parser.stats = EthRxStats()

    def _check_uplink_stale(self) -> None:
        if self._last_blob_uplink_time <= 0:
            return
        if time.monotonic() - self._last_blob_uplink_time > self._uplink_timeout_s:
            self.get_logger().warn(
                f"uplink timeout (>{self._uplink_timeout_s * 1000:.0f}ms no BLOB uplink)",
                throttle_duration_sec=5.0,
            )

    def destroy_node(self) -> bool:
        self._shutting_down = True
        self._stop_rx_thread()
        if self._time_sync_enable and self._link.connected:
            try:
                self._link.send(self._time_sync.stop_session())
                self.get_logger().info("TimeSync STOP sent")
            except OSError:
                pass
        self._link.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = EthGatewayNode()
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
