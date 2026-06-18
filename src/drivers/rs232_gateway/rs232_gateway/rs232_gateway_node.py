#!/usr/bin/env python3
"""RS232 gateway: V3 / BLOB v2 串口 ↔ jetson_can_msgs topics + TimeSync。"""

from __future__ import annotations

import queue
import threading
import time

import rclpy
from jetson_can_msgs.msg import (
    BlobSensorCfg,
    FaultReport,
    GpsFrameA,
    GpsFrameB,
    GpsFrameC,
    StatusSnapshot,
    TimeSyncResponse,
    V3Command,
    V3ExtStatus,
    V3Status,
)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8MultiArray

# launch 传 tx_rate_hz:=20 时为 INTEGER，需 dynamic_typing
_PARAM_FLOAT = ParameterDescriptor(dynamic_typing=True)


def _param_float(node: Node, name: str) -> float:
    return float(node.get_parameter(name).value)

from ds_jetson_bridge.jetson_protocol import (
    FRAME_TYPE_UP_EXT,
    FRAME_TYPE_UP_STATUS,
    UPLINK_TIMEOUT_MS,
    encode_stop_downlink,
    parse_uplink_ext,
    parse_uplink_status,
)

from rs232_gateway.blob_codec import encode_agv_control, encode_sensor_cfg, v3_command_to_blob
from rs232_gateway.blob_topic_pub import BlobTopicPublisher
from rs232_gateway.serial_io import SerialLink
from rs232_gateway.service_codec import (
    payload_to_fault,
    payload_to_gps_a,
    payload_to_gps_b,
    payload_to_gps_c,
    payload_to_status_snapshot,
    payload_to_time_sync,
)
from rs232_gateway.service_frame import (
    CAN_ID_FAULT,
    CAN_ID_GPS_A,
    CAN_ID_GPS_B,
    CAN_ID_GPS_C,
    CAN_ID_STATUS_SNAPSHOT,
    CAN_ID_TIME_SYNC_RSP,
    Rs232StreamParser,
    RxStats,
    StreamItem,
)
from rs232_gateway.time_sync import CMD_START, JetsonTimeSync, mono_ms
from rs232_gateway.v3_codec import (
    uplink_ext_to_msg,
    uplink_status_to_msg,
    v3_command_to_bytes,
)


class Rs232GatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("rs232_gateway")

        self.declare_parameter(
            "serial_port",
            "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0",
        )
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("tx_rate_hz", 50.0, _PARAM_FLOAT)
        self.declare_parameter("uplink_timeout_ms", UPLINK_TIMEOUT_MS)
        self.declare_parameter("auto_reconnect", True)
        self.declare_parameter("reconnect_interval_s", 1.0, _PARAM_FLOAT)
        self.declare_parameter("reconnect_settle_s", 0.15, _PARAM_FLOAT)
        self.declare_parameter("publish_raw", False)
        self.declare_parameter("heartbeat_mode_req", 1)
        self.declare_parameter("time_sync_enable", True)
        self.declare_parameter("time_sync_session_id", 1)
        self.declare_parameter("time_sync_ping_burst", 10)
        self.declare_parameter("time_sync_ping_burst_interval_s", 0.1, _PARAM_FLOAT)
        self.declare_parameter("time_sync_ping_interval_s", 1.0, _PARAM_FLOAT)
        self.declare_parameter("time_sync_query_interval_s", 10.0, _PARAM_FLOAT)
        self.declare_parameter("time_sync_rtt_warn_ms", 50.0, _PARAM_FLOAT)
        self.declare_parameter("use_blob_v2", True)
        self.declare_parameter("debug_rx_stats_interval_s", 5.0, _PARAM_FLOAT)
        self.declare_parameter("skip_dtr_reset", True)
        self.declare_parameter("no_flush_on_open", True)
        self.declare_parameter("flush_rx_on_connect", False)
        self.declare_parameter("rx_dispatch_hz", 50.0, _PARAM_FLOAT)

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self._tx_rate_hz = _param_float(self, "tx_rate_hz")
        self._uplink_timeout_s = (
            self.get_parameter("uplink_timeout_ms").get_parameter_value().integer_value
            / 1000.0
        )
        self._auto_reconnect = (
            self.get_parameter("auto_reconnect").get_parameter_value().bool_value
        )
        self._reconnect_interval_s = _param_float(self, "reconnect_interval_s")
        settle = _param_float(self, "reconnect_settle_s")
        self._publish_raw = self.get_parameter("publish_raw").get_parameter_value().bool_value
        self._heartbeat_mode_req = (
            self.get_parameter("heartbeat_mode_req").get_parameter_value().integer_value
        )
        self._time_sync_enable = (
            self.get_parameter("time_sync_enable").get_parameter_value().bool_value
        )
        self._use_blob_v2 = (
            self.get_parameter("use_blob_v2").get_parameter_value().bool_value
        )
        self._debug_rx_stats_interval_s = _param_float(self, "debug_rx_stats_interval_s")
        skip_dtr = self.get_parameter("skip_dtr_reset").get_parameter_value().bool_value
        no_flush = self.get_parameter("no_flush_on_open").get_parameter_value().bool_value
        self._flush_rx_on_connect = (
            self.get_parameter("flush_rx_on_connect").get_parameter_value().bool_value
        )
        rx_dispatch_hz = _param_float(self, "rx_dispatch_hz")

        self._time_sync = JetsonTimeSync(
            session_id=self.get_parameter("time_sync_session_id").get_parameter_value().integer_value,
            ping_burst_count=self.get_parameter("time_sync_ping_burst").get_parameter_value().integer_value,
            ping_burst_interval_s=_param_float(self, "time_sync_ping_burst_interval_s"),
            ping_interval_s=_param_float(self, "time_sync_ping_interval_s"),
            query_interval_s=_param_float(self, "time_sync_query_interval_s"),
            rtt_warn_ms=_param_float(self, "time_sync_rtt_warn_ms"),
        )

        self._link = SerialLink(
            port,
            baud,
            settle_s=settle,
            skip_dtr_reset=skip_dtr,
            no_flush_on_open=no_flush,
            logger=self.get_logger(),
        )
        self._parser = Rs232StreamParser(parse_v3=not self._use_blob_v2)
        self._tx_seq = 0
        self._last_uplink_time = 0.0
        self._last_blob_uplink_time = 0.0
        self._next_reconnect = 0.0
        self._latest_command: V3Command | None = None
        self._link_was_connected = False
        self._blob_pub = BlobTopicPublisher(self, "/jetson_rs232")
        self._cfg_tx_seq = 0
        self._pub_status_interval = 0
        self._rx_queue: queue.Queue[StreamItem] = queue.Queue(maxsize=2048)
        self._rx_stop = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._last_stats_log = time.monotonic()

        self._pub_link = self.create_publisher(Bool, "/jetson_rs232/link", 10)
        self._pub_time_sync = self.create_publisher(
            TimeSyncResponse, "/jetson_rs232/time_sync", 10
        )
        self._pub_fault = self.create_publisher(FaultReport, "/jetson_rs232/fault", 10)
        self._pub_snapshot = self.create_publisher(
            StatusSnapshot, "/jetson_rs232/status_snapshot", 10
        )
        if self._publish_raw:
            self._pub_raw = self.create_publisher(
                UInt8MultiArray, "/jetson_rs232/raw_rx", 10
            )
        else:
            self._pub_raw = None

        self._sub_command = self.create_subscription(
            V3Command, "/jetson_rs232/command", self._command_cb, 10
        )
        self._sub_sensor_cfg = self.create_subscription(
            BlobSensorCfg,
            "/jetson_rs232/blob/sensor_cfg",
            self._sensor_cfg_cb,
            10,
        )

        period = 1.0 / self._tx_rate_hz if self._tx_rate_hz > 0 else 0.02
        self._timer = self.create_timer(period, self._tick)
        rx_period = 1.0 / rx_dispatch_hz if rx_dispatch_hz > 0 else 0.02
        self._rx_dispatch_timer = self.create_timer(rx_period, self._dispatch_rx_only)

        if not self._ensure_serial():
            self.get_logger().warn(
                f"串口暂未打开 ({port})，将按 {self._reconnect_interval_s}s 重试"
            )

        self.get_logger().info(
            f"rs232_gateway @ {port} {baud}bps, {self._tx_rate_hz:.0f}Hz, "
            f"protocol={'BLOB v2' if self._use_blob_v2 else 'V3 0xAA'}, "
            f"time_sync={'on' if self._time_sync_enable else 'off'}, "
            f"RX=独立线程, topics /jetson_rs232/{{v3_*,gps/*,command,link,...}}"
        )

    def _command_cb(self, msg: V3Command) -> None:
        self._latest_command = msg

    def _sensor_cfg_cb(self, msg: BlobSensorCfg) -> None:
        if not self._link.connected or not self._use_blob_v2:
            return
        ts = msg.timestamp_ms if msg.timestamp_ms else int(mono_ms())
        try:
            frame = encode_sensor_cfg(
                self._cfg_tx_seq,
                ts,
                threshold_mm=msg.threshold_mm,
                enable_mask=msg.enable_mask,
            )
            self._link.write(frame)
            self._cfg_tx_seq = (self._cfg_tx_seq + 1) & 0xFF
        except OSError as exc:
            self.get_logger().warn(f"sensor_cfg 写入失败: {exc}")

    def _start_rx_thread(self) -> None:
        self._stop_rx_thread()
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="rs232_rx", daemon=True
        )
        self._rx_thread.start()

    def _stop_rx_thread(self) -> None:
        self._rx_stop.set()
        if self._rx_thread is not None and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        self._rx_thread = None
        self._process_rx_queue()
        self._drain_rx_queue()

    def _drain_rx_queue(self) -> None:
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
            try:
                raw = self._link.read_available()
            except OSError:
                time.sleep(0.01)
                continue
            if raw:
                for item in self._parser.feed(raw):
                    try:
                        self._rx_queue.put(item, timeout=0.05)
                    except queue.Full:
                        self.get_logger().warn(
                            "RX 队列满，丢弃一帧",
                            throttle_duration_sec=5.0,
                        )
            else:
                time.sleep(0.002)

    def _ensure_serial(self) -> bool:
        if self._link.connected:
            return True
        if not self._auto_reconnect:
            return False
        now = time.monotonic()
        if now < self._next_reconnect:
            return False
        self._next_reconnect = now + self._reconnect_interval_s
        self._stop_rx_thread()
        if self._link.open():
            self._parser = Rs232StreamParser(parse_v3=not self._use_blob_v2)
            self._blob_pub.reset_cache()
            self._last_uplink_time = 0.0
            self._last_blob_uplink_time = 0.0
            self._last_stats_log = time.monotonic()
            self.get_logger().info(f"串口已连接: {self._link.port}")
            if self._flush_rx_on_connect:
                self._link.flush_rx()
            self._start_rx_thread()
            self._prime_downlink()
            return True
        return False

    def _prime_downlink(self, frames: int = 10) -> None:
        if not self._link.connected:
            return
        for _ in range(frames):
            try:
                if self._use_blob_v2:
                    frame = encode_agv_control(
                        self._tx_seq, int(mono_ms()), control_mode=self._heartbeat_mode_req
                    )
                else:
                    frame = encode_stop_downlink(
                        self._tx_seq, mode_req=self._heartbeat_mode_req
                    )
                self._link.write(frame)
                self._tx_seq = (self._tx_seq + 1) & 0xFF
            except OSError:
                self._link.close()
                break
            time.sleep(0.02)
        if self._time_sync_enable:
            try:
                self._link.write(self._time_sync.begin_session())
                self.get_logger().info(
                    f"TimeSync START session={self._time_sync.session_id}, "
                    f"burst={self._time_sync.ping_burst_count}"
                )
            except OSError:
                self._link.close()

    def _write_service_frames(self, frames: list[bytes]) -> None:
        for frame in frames:
            self._link.write(frame)

    def _publish_link(self, connected: bool) -> None:
        msg = Bool()
        msg.data = connected
        self._pub_link.publish(msg)
        if connected != self._link_was_connected:
            self.get_logger().info("链路: 已连接" if connected else "链路: 断开")
            self._link_was_connected = connected

    def _dispatch_rx_only(self) -> None:
        self._process_rx_queue()

    def _tick(self) -> None:
        connected = self._ensure_serial()
        self._publish_link(connected)
        self._process_rx_queue()
        if not connected:
            return

        try:
            if self._use_blob_v2:
                frame = v3_command_to_blob(
                    self._tx_seq, self._latest_command, int(mono_ms())
                )
            else:
                frame = v3_command_to_bytes(self._tx_seq, self._latest_command)
            self._link.write(frame)
            self._tx_seq = (self._tx_seq + 1) & 0xFF
            if self._time_sync_enable:
                self._write_service_frames(self._time_sync.tick())
        except OSError as exc:
            self.get_logger().warn(f"串口写入失败: {exc}")
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
            self._dispatch_item(item, stamp)

    def _dispatch_item(self, item: StreamItem, stamp) -> None:
        if item[0] == "blob":
            _, msg_id, blob_seq, payload = item
            self._handle_blob_frame(msg_id, blob_seq, payload, stamp)
        elif item[0] == "v3":
            v3_frame = item[1]
            if self._pub_raw is not None:
                raw_msg = UInt8MultiArray()
                raw_msg.data = list(v3_frame)
                self._pub_raw.publish(raw_msg)

            if v3_frame[1] == FRAME_TYPE_UP_STATUS:
                status = parse_uplink_status(v3_frame)
                if status is None:
                    return
                self._last_uplink_time = time.monotonic()
                self._blob_pub.pub_v3_status.publish(uplink_status_to_msg(status, stamp))
                self._pub_status_interval += 1
            elif v3_frame[1] == FRAME_TYPE_UP_EXT:
                ext = parse_uplink_ext(v3_frame)
                if ext is None:
                    return
                self._last_uplink_time = time.monotonic()
                self._blob_pub.pub_v3_ext.publish(uplink_ext_to_msg(ext, stamp))
        else:
            _, can_id, payload = item
            if can_id == CAN_ID_TIME_SYNC_RSP:
                self._handle_time_sync(payload, stamp, t4_ms=mono_ms())
            else:
                self._handle_service_frame(can_id, payload, stamp)

    def _handle_blob_frame(
        self, msg_id: int, blob_seq: int, payload: bytes, stamp
    ) -> None:
        if self._blob_pub.handle_frame(msg_id, blob_seq, payload, stamp):
            self._last_blob_uplink_time = time.monotonic()
            self._pub_status_interval += 1

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
                            f"TimeSync RTT 过高: {update.rtt_ms:.2f}ms "
                            f"(阈值>{self._time_sync.rtt_warn_ms:.0f}ms)",
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

    def _handle_service_frame(self, can_id: int, payload: bytes, stamp) -> None:
        if can_id == CAN_ID_GPS_A:
            msg = payload_to_gps_a(payload, stamp)
            if msg is not None:
                self._blob_pub.pub_gps_a.publish(msg)
        elif can_id == CAN_ID_GPS_B:
            msg = payload_to_gps_b(payload, stamp)
            if msg is not None:
                self._blob_pub.pub_gps_b.publish(msg)
        elif can_id == CAN_ID_GPS_C:
            msg = payload_to_gps_c(payload, stamp)
            if msg is not None:
                self._blob_pub.pub_gps_c.publish(msg)
        elif can_id == CAN_ID_FAULT:
            self._pub_fault.publish(payload_to_fault(payload, stamp))
        elif can_id == CAN_ID_STATUS_SNAPSHOT:
            self._pub_snapshot.publish(payload_to_status_snapshot(payload, stamp))

    def _maybe_log_rx_stats(self) -> None:
        if self._debug_rx_stats_interval_s <= 0:
            return
        now = time.monotonic()
        if now - self._last_stats_log < self._debug_rx_stats_interval_s:
            return
        self._last_stats_log = now
        s = self._parser.stats.snapshot()
        self.get_logger().info(
            "RX +{:.0f}s: bytes={} blob02={} blob03={} blob04={} "
            "blob_other={} svc={} v3={} hdr_rej={} resync={} pub_status={}".format(
                self._debug_rx_stats_interval_s,
                s["bytes_in"],
                s["blob_02"],
                s["blob_03"],
                s["blob_04"],
                s["blob_other"],
                s["svc_rx"],
                s["v3_rx"],
                s["hdr_reject"],
                s["resync"],
                self._pub_status_interval,
            )
        )
        self._pub_status_interval = 0
        self._parser.stats = RxStats()

    def _check_uplink_stale(self) -> None:
        if self._use_blob_v2:
            last = self._last_blob_uplink_time
            label = " BLOB uplink"
        else:
            last = self._last_uplink_time
            label = " 0x02/0x03"
        if last <= 0:
            return
        if time.monotonic() - last > self._uplink_timeout_s:
            self.get_logger().warn(
                f"上行超时 (>{self._uplink_timeout_s * 1000:.0f}ms 无有效{label})",
                throttle_duration_sec=5.0,
            )

    def destroy_node(self) -> bool:
        self._stop_rx_thread()
        if self._time_sync_enable and self._link.connected:
            try:
                self._link.write(self._time_sync.stop_session())
                self.get_logger().info("TimeSync STOP 已发送")
            except OSError:
                pass
        self._link.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Rs232GatewayNode()
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
