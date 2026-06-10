#!/usr/bin/env python3
"""Jetson bridge V3: 24B downlink to STM32B, parse 0x02 uplink, /cmd_vel in."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String, UInt8, UInt16MultiArray

try:
    import serial
except ImportError as exc:
    raise RuntimeError("Install: sudo apt install python3-serial") from exc

from ds_jetson_bridge.jetson_protocol import (
    CMD_TIMEOUT_MS,
    MODE_CAN,
    RECOVER_STABLE_MS,
    UPLINK_TIMEOUT_MS,
    DownlinkCommand,
    FrameParser,
    encode_downlink,
    encode_stop_downlink,
    format_bridge_local_status,
    format_downlink_log,
    format_status_echo,
    format_uplink_log,
    parse_uplink_status,
    twist_to_motion,
)


class JetsonBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("jetson_bridge")

        self.declare_parameter("cmd_port", "/dev/ttyUSB5")
        self.declare_parameter("cmd_baud", 115200)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("cmd_timeout_ms", CMD_TIMEOUT_MS)
        self.declare_parameter("max_linear_m_s", 0.8)
        self.declare_parameter("max_angular_rad_s", 1.0)
        self.declare_parameter("cruise_scale", 0.7)
        self.declare_parameter("mode_req", MODE_CAN)
        self.declare_parameter("motion_model", 0)
        self.declare_parameter("strafe_jl_from_angular", True)
        self.declare_parameter("strafe_speed_m_s", 0.3)
        self.declare_parameter("sideways_steer_millirad", 1571)
        self.declare_parameter("mode_hold_frames", 50)
        self.declare_parameter("log_tx", True)
        self.declare_parameter("auto_reconnect", True)
        self.declare_parameter("reconnect_interval_s", 1.0)
        self.declare_parameter("reconnect_settle_s", 0.15)
        self.declare_parameter("reconnect_uplink_deadline_s", 2.0)

        self._port = self.get_parameter("cmd_port").get_parameter_value().string_value
        baud = self.get_parameter("cmd_baud").get_parameter_value().integer_value
        self._baud = baud
        self._rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self._cmd_timeout_s = (
            self.get_parameter("cmd_timeout_ms").get_parameter_value().integer_value
            / 1000.0
        )
        self._max_linear = (
            self.get_parameter("max_linear_m_s").get_parameter_value().double_value
        )
        self._max_angular = (
            self.get_parameter("max_angular_rad_s").get_parameter_value().double_value
        )
        self._cruise_scale = (
            self.get_parameter("cruise_scale").get_parameter_value().double_value
        )
        self._mode_req = (
            self.get_parameter("mode_req").get_parameter_value().integer_value
        )
        self._default_motion_model = (
            self.get_parameter("motion_model").get_parameter_value().integer_value
        )
        self._strafe_jl = (
            self.get_parameter("strafe_jl_from_angular")
            .get_parameter_value()
            .bool_value
        )
        self._strafe_speed_m_s = (
            self.get_parameter("strafe_speed_m_s").get_parameter_value().double_value
        )
        self._sideways_steer_millirad = (
            self.get_parameter("sideways_steer_millirad")
            .get_parameter_value()
            .integer_value
        )
        self._mode_hold_frames = (
            self.get_parameter("mode_hold_frames").get_parameter_value().integer_value
        )
        self._log_tx = self.get_parameter("log_tx").get_parameter_value().bool_value
        self._auto_reconnect = (
            self.get_parameter("auto_reconnect").get_parameter_value().bool_value
        )
        self._reconnect_interval_s = (
            self.get_parameter("reconnect_interval_s")
            .get_parameter_value()
            .double_value
        )
        self._reconnect_settle_s = (
            self.get_parameter("reconnect_settle_s")
            .get_parameter_value()
            .double_value
        )
        self._reconnect_uplink_deadline_s = (
            self.get_parameter("reconnect_uplink_deadline_s")
            .get_parameter_value()
            .double_value
        )

        self._ser: serial.Serial | None = None
        self._next_reconnect_time = 0.0
        self._last_serial_close_time = 0.0
        self._serial_restored_at = 0.0
        if not self._open_serial():
            hint = (
                f"无法打开串口 {self._port}. "
                "常见原因：STM32B 的 USB 线未插好、pl2303 驱动僵死、或口被占用。"
                "请执行 ls -l /dev/serial/by-id/* 确认 Prolific 设备，"
                "拔掉 USB 等 3 秒再插上，或换 cmd_port:=/dev/serial/by-id/usb-Prolific_...-port0"
            )
            raise RuntimeError(hint)

        self._parser = FrameParser()
        self._tx_seq = 0
        self._tx_count = 0
        self._last_cmd_time = time.monotonic()
        self._had_cmd_vel = False
        self._pending_v_mm_s = 0
        self._pending_omega = 0
        self._pending_steer = 0
        self._pending_motion_model = self._default_motion_model
        self._last_motion_v_mm_s = 0
        self._last_motion_omega = 0
        self._last_motion_steer = 0
        self._recover_until = 0.0
        self._last_uplink_time = 0.0
        self._last_status = None
        self._last_local_status_publish = 0.0
        self._serial_disconnected_since = 0.0

        self._sub = self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

        self._pub_safety = self.create_publisher(UInt8, "stm32b/safety_state", 10)
        self._pub_limit = self.create_publisher(UInt8, "stm32b/limit_factor", 10)
        self._pub_link = self.create_publisher(UInt8, "stm32b/link_state", 10)
        self._pub_motion = self.create_publisher(Twist, "stm32b/motion_actual", 10)
        self._pub_sonar = self.create_publisher(UInt16MultiArray, "stm32b/sonar_mm", 10)
        self._pub_batt_v = self.create_publisher(Float32, "stm32b/battery_voltage", 10)
        self._pub_batt_soc = self.create_publisher(UInt8, "stm32b/battery_soc", 10)
        self._pub_uplink_seq = self.create_publisher(UInt8, "stm32b/uplink_seq", 10)
        self._pub_status = self.create_publisher(String, "stm32b/status", 10)

        period = 1.0 / self._rate_hz if self._rate_hz > 0 else 0.02
        self._timer = self.create_timer(period, self._tick)

        jl_mode = "j/l→斜移" if self._strafe_jl else "j/l→自旋"
        reconnect = "on" if self._auto_reconnect else "off"
        self.get_logger().info(
            f"Jetson bridge V3 on {self._port} @ {self._baud}, {self._rate_hz:.0f}Hz, "
            f"24B 0xAA down + 0x02 up, {jl_mode}, serial reconnect={reconnect}"
        )

    def _open_serial(self) -> bool:
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
                dsrdtr=False,
                rtscts=False,
            )
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            # Prolific 热插拔后有时需 DTR 翻转才能恢复 RX
            try:
                self._ser.dtr = False
                time.sleep(0.05)
                self._ser.dtr = True
            except Exception:
                pass
            time.sleep(self._reconnect_settle_s)
            return True
        except OSError as exc:
            self._ser = None
            self.get_logger().warn(
                f"Serial open failed ({self._port}): {exc}",
                throttle_duration_sec=5.0,
            )
            return False

    def _flush_rx_garbage(self, discard_s: float = 0.4) -> int:
        """Drop stale bytes after hotplug (do not parse)."""
        if self._ser is None:
            return 0
        discarded = 0
        old_timeout = self._ser.timeout
        self._ser.timeout = 0.05
        deadline = time.monotonic() + discard_s
        try:
            while time.monotonic() < deadline:
                chunk = self._ser.read(512)
                if chunk:
                    discarded += len(chunk)
                elif self._ser.in_waiting:
                    continue
                else:
                    time.sleep(0.01)
        except (OSError, serial.SerialException):
            pass
        finally:
            self._ser.timeout = old_timeout
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass
        return discarded

    def _prime_downlink(self, frames: int = 15) -> None:
        """Send a short burst so STM32B sees fresh seq after reconnect."""
        if self._ser is None:
            return
        for _ in range(frames):
            try:
                frame = encode_downlink(
                    self._tx_seq,
                    DownlinkCommand(mode_req=MODE_CAN),
                )
                self._ser.write(frame)
                self._tx_seq = (self._tx_seq + 1) & 0xFF
            except (OSError, serial.SerialException):
                break
            time.sleep(0.02)

    def _read_uplink_window(self, window_s: float = 0.6) -> int:
        """Parse 0x02 uplink for a short window after reconnect."""
        if self._ser is None:
            return 0
        raw = 0
        old_timeout = self._ser.timeout
        self._ser.timeout = 0.05
        deadline = time.monotonic() + window_s
        try:
            while time.monotonic() < deadline:
                chunk = self._ser.read(512)
                if chunk:
                    raw += len(chunk)
                    for frame in self._parser.feed(chunk):
                        status = parse_uplink_status(frame)
                        if status is not None:
                            self._last_uplink_time = time.monotonic()
                            self._last_status = status
                            self._serial_restored_at = 0.0
                            self._publish_status(status)
                            return raw
                elif self._ser.in_waiting:
                    continue
                else:
                    time.sleep(0.01)
        except (OSError, serial.SerialException):
            pass
        finally:
            self._ser.timeout = old_timeout
        return raw

    def _bringup_serial_link(self, *, read_window_s: float = 0.6) -> tuple[int, int, bool]:
        """Flush garbage, prime TX, read RX. Returns (discarded, read, ok)."""
        discarded = self._flush_rx_garbage()
        self._parser = FrameParser()
        self._prime_downlink()
        read = self._read_uplink_window(window_s=read_window_s)
        ok = self._last_uplink_time > 0
        return discarded, read, ok

    def _close_serial(self) -> None:
        if self._ser is None:
            return
        try:
            self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._last_serial_close_time = time.monotonic()

    def _hard_reconnect(self, reason: str) -> None:
        self.get_logger().warn(
            f"Hard serial reconnect: {reason}",
            throttle_duration_sec=3.0,
        )
        self._close_serial()
        self._next_reconnect_time = 0.0

    def _on_serial_restored(self) -> None:
        self._parser = FrameParser()
        self._last_uplink_time = 0.0
        self._last_status = None
        self._serial_disconnected_since = 0.0
        self._serial_restored_at = time.monotonic()
        self._tx_count = 0

        total_discarded = 0
        total_read = 0
        for attempt in (1, 2):
            read_window = 0.6 if attempt == 1 else 1.0
            discarded, read, ok = self._bringup_serial_link(
                read_window_s=read_window
            )
            total_discarded += discarded
            total_read += read
            if ok:
                self.get_logger().info(
                    f"Serial restored on {self._port} (try {attempt}); "
                    f"flush={total_discarded}B read={total_read}B, 0x02 uplink OK"
                )
                return
            if attempt == 1:
                time.sleep(0.15)

        self.get_logger().info(
            f"Serial restored on {self._port}; flush={total_discarded}B "
            f"read={total_read}B, no valid 0x02 — scheduling hard reconnect"
        )
        self._publish_local_status(waiting_uplink=True)
        self._hard_reconnect("no valid 0x02 after reopen")

    def _ensure_serial(self) -> bool:
        if self._ser is not None and self._ser.is_open:
            return True
        if not self._auto_reconnect:
            return False
        now = time.monotonic()
        if (now - self._last_serial_close_time) < self._reconnect_settle_s:
            return False
        if now < self._next_reconnect_time:
            return False
        self._next_reconnect_time = now + self._reconnect_interval_s
        if self._open_serial():
            self._on_serial_restored()
        return self._ser is not None and self._ser.is_open

    def _handle_serial_error(self, exc: Exception) -> None:
        self.get_logger().warn(
            f"Serial I/O lost ({self._port}): {exc}; "
            "unplug/replug USB then wait for auto-reconnect",
            throttle_duration_sec=3.0,
        )
        if self._serial_disconnected_since <= 0.0:
            self._serial_disconnected_since = time.monotonic()
        self._close_serial()
        self._publish_local_status(serial_ok=False)

    def _publish_local_status(
        self,
        *,
        serial_ok: bool = True,
        waiting_uplink: bool = False,
        uplink_stale: bool = False,
    ) -> None:
        msg = String()
        msg.data = format_bridge_local_status(
            serial_ok=serial_ok,
            waiting_uplink=waiting_uplink,
            uplink_stale=uplink_stale,
        )
        self._pub_status.publish(msg)
        self._pub_link.publish(UInt8(data=0x01 if not serial_ok else 0x00))
        self._last_local_status_publish = time.monotonic()

    def _maybe_publish_local_status(
        self,
        *,
        serial_ok: bool,
        waiting_uplink: bool = False,
        uplink_stale: bool = False,
        interval_s: float = 0.5,
    ) -> None:
        now = time.monotonic()
        if (now - self._last_local_status_publish) < interval_s:
            return
        self._publish_local_status(
            serial_ok=serial_ok,
            waiting_uplink=waiting_uplink,
            uplink_stale=uplink_stale,
        )

    def _cmd_vel_cb(self, msg: Twist) -> None:
        lx = msg.linear.x * self._cruise_scale
        ly = msg.linear.y * self._cruise_scale
        az = msg.angular.z * self._cruise_scale
        lx = max(-self._max_linear, min(self._max_linear, lx))
        ly = max(-self._max_linear, min(self._max_linear, ly))
        az = max(-self._max_angular, min(self._max_angular, az))

        motion = twist_to_motion(
            lx,
            ly,
            az,
            strafe_jl_from_angular=self._strafe_jl,
            strafe_speed_m_s=self._strafe_speed_m_s,
            sideways_steer_millirad=self._sideways_steer_millirad,
        )
        self._pending_v_mm_s = motion.v_mm_s
        self._pending_omega = motion.omega_millirad_s
        self._pending_steer = motion.steer_millirad
        self._pending_motion_model = motion.motion_model
        self._last_cmd_time = time.monotonic()
        self._had_cmd_vel = True
        if (
            self._pending_v_mm_s != 0
            or self._pending_omega != 0
            or self._pending_steer != 0
        ):
            self._last_motion_v_mm_s = self._pending_v_mm_s
            self._last_motion_omega = self._pending_omega
            self._last_motion_steer = self._pending_steer

    def _in_recovery(self) -> bool:
        return time.monotonic() < self._recover_until

    def _start_recovery(self) -> None:
        self._recover_until = time.monotonic() + RECOVER_STABLE_MS / 1000.0
        self._pending_v_mm_s = 0
        self._pending_omega = 0
        self._pending_steer = 0
        self._pending_motion_model = self._default_motion_model

    def _mode_for_tick(self) -> int:
        if self._tx_count < self._mode_hold_frames:
            return MODE_CAN
        return self._mode_req

    def _build_downlink(self) -> bytes:
        now = time.monotonic()
        cmd_stale = (now - self._last_cmd_time) > self._cmd_timeout_s

        had_recent_motion = (
            self._last_motion_v_mm_s != 0
            or self._last_motion_omega != 0
            or self._last_motion_steer != 0
        )
        if (
            had_recent_motion
            and cmd_stale
            and not self._in_recovery()
        ):
            self._start_recovery()
            self.get_logger().warn(
                "cmd_vel lost after motion -> 1s zero (recovery)",
                throttle_duration_sec=2.0,
            )

        allow_motion = (
            not self._in_recovery()
            and not cmd_stale
            and (
                self._pending_v_mm_s != 0
                or self._pending_omega != 0
                or self._pending_steer != 0
            )
        )

        steer = 0
        motion_model = self._default_motion_model
        if self._in_recovery():
            if now >= self._recover_until and not cmd_stale:
                self._recover_until = 0.0
                self.get_logger().info("Recovery done, accepting cmd_vel")
            v_mm_s, omega = 0, 0
            mode = "RECOVER"
        elif not allow_motion:
            v_mm_s, omega = 0, 0
            mode = "IDLE"
        else:
            v_mm_s = self._pending_v_mm_s
            omega = self._pending_omega
            steer = self._pending_steer
            motion_model = self._pending_motion_model
            mode = "RUN"

        cmd = DownlinkCommand(
            mode_req=self._mode_for_tick(),
            v_mm_s=v_mm_s,
            omega_millirad_s=omega,
            steer_millirad=steer,
            motion_model=motion_model,
        )
        frame = encode_downlink(self._tx_seq, cmd)
        self._last_mode = mode
        return frame

    def _poll_uplink(self) -> None:
        if self._ser is None:
            return
        try:
            n = self._ser.in_waiting
            if n <= 0:
                return
            chunk = self._ser.read(n)
        except (OSError, serial.SerialException) as exc:
            self._handle_serial_error(exc)
            return
        for raw in self._parser.feed(chunk):
            status = parse_uplink_status(raw)
            if status is None:
                continue
            self._last_uplink_time = time.monotonic()
            self._last_status = status
            self._serial_restored_at = 0.0
            self._publish_status(status)

    def _publish_status(self, s) -> None:
        self._pub_safety.publish(UInt8(data=s.safety_state))
        self._pub_limit.publish(UInt8(data=s.limit_factor))
        self._pub_link.publish(UInt8(data=s.link_state))
        self._pub_uplink_seq.publish(UInt8(data=s.seq & 0xFF))
        self._pub_batt_soc.publish(UInt8(data=s.battery_soc))

        bv = Float32()
        bv.data = s.battery_voltage_0p1v / 10.0
        self._pub_batt_v.publish(bv)

        motion = Twist()
        motion.linear.x = s.v_actual_mm_s / 1000.0
        motion.angular.z = s.omega_millirad_s / 1000.0
        self._pub_motion.publish(motion)

        sonar = UInt16MultiArray()
        sonar.data = [
            s.sonar_front_mm or 0,
            s.sonar_back_mm or 0,
            s.sonar_left_mm or 0,
            s.sonar_right_mm or 0,
        ]
        self._pub_sonar.publish(sonar)

        status_msg = String()
        status_msg.data = format_status_echo(s)
        self._pub_status.publish(status_msg)

    def _tick(self) -> None:
        if not self._ensure_serial() or self._ser is None:
            if self._serial_disconnected_since <= 0.0:
                self._serial_disconnected_since = time.monotonic()
            self._maybe_publish_local_status(serial_ok=False)
            return

        frame = self._build_downlink()
        try:
            self._ser.write(frame)
        except (OSError, serial.SerialException) as exc:
            self._handle_serial_error(exc)
            return

        self._tx_seq = (self._tx_seq + 1) & 0xFF
        self._tx_count += 1

        self._poll_uplink()

        now = time.monotonic()
        if (
            self._serial_restored_at > 0
            and self._last_uplink_time <= 0
            and (now - self._serial_restored_at)
            > self._reconnect_uplink_deadline_s
        ):
            self._hard_reconnect(
                f"no 0x02 uplink within {self._reconnect_uplink_deadline_s:.0f}s after reopen"
            )
            return

        if self._log_tx and self._tx_count % max(1, int(self._rate_hz)) == 0:
            bridge_mode = getattr(self, "_last_mode", "?")
            self.get_logger().info(
                f"TX {format_downlink_log(frame, bridge_mode)}",
                throttle_duration_sec=1.0,
            )

        if self._last_status and self._tx_count % max(1, int(self._rate_hz)) == 0:
            self.get_logger().info(
                f"RX {format_uplink_log(self._last_status)}",
                throttle_duration_sec=1.0,
            )

        now = time.monotonic()
        uplink_stale = (
            self._last_uplink_time > 0
            and (now - self._last_uplink_time) > UPLINK_TIMEOUT_MS / 1000.0
        )
        waiting_uplink = (
            self._last_uplink_time == 0 and self._tx_count > int(self._rate_hz * 3)
        )
        if uplink_stale:
            self.get_logger().warn(
                "No valid 0x02 uplink from STM32B",
                throttle_duration_sec=3.0,
            )
            self._maybe_publish_local_status(
                serial_ok=True, uplink_stale=True
            )
        elif waiting_uplink:
            self.get_logger().warn(
                "Waiting for STM32B 0x02 uplink (firmware ready?)",
                throttle_duration_sec=5.0,
            )
            self._maybe_publish_local_status(
                serial_ok=True, waiting_uplink=True
            )

    def destroy_node(self) -> bool:
        try:
            if self._ser is not None and self._ser.is_open:
                self._ser.write(encode_stop_downlink(self._tx_seq, MODE_CAN))
        except Exception:
            pass
        self._close_serial()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = JetsonBridgeNode()
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
