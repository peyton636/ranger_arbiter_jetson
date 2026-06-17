#!/usr/bin/env python3
"""
时间同步 1h soak 监测（需 rs232_gateway 在跑）。

订阅 /jetson_rs232/time_sync 与 /jetson_rs232/v3_status，统计：
  - offset_ms 漂移（min/max/标准差）
  - RTT 分布与 >50ms WARN 比例
  - V3 上行近似频率

用法：
  # 终端1: ros2 launch rs232_gateway rs232_gateway.launch.py serial_port:=/dev/ttyUSB7
  # 终端2:
  python3 tools/jetson_time_soak.py --duration 3600
  python3 tools/jetson_time_soak.py --duration 300   # 5 分钟试跑
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

import rclpy
from jetson_can_msgs.msg import TimeSyncResponse, V3Status
from rclpy.node import Node

CMD_PING = 0x03
RTT_WARN_MS = 50.0


class TimeSoakNode(Node):
    def __init__(self, duration_s: float, rtt_warn_ms: float) -> None:
        super().__init__("jetson_time_soak")
        self._duration_s = duration_s
        self._rtt_warn_ms = rtt_warn_ms
        self._start = time.monotonic()
        self._deadline = self._start + duration_s

        self._offsets: list[float] = []
        self._rtts: list[float] = []
        self._ping_count = 0
        self._query_count = 0
        self._rtt_warn_count = 0
        self._v3_count = 0
        self._time_sync_any = 0
        self._last_report = self._start
        self._no_data_warned = False

        self.create_subscription(
            TimeSyncResponse,
            "/jetson_rs232/time_sync",
            self._time_sync_cb,
            10,
        )
        self.create_subscription(
            V3Status, "/jetson_rs232/v3_status", self._v3_cb, 10
        )
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"soak 开始: 时长={duration_s:.0f}s, RTT告警阈值={rtt_warn_ms:.0f}ms"
        )
        self.get_logger().info(
            "请保持另一终端 rs232_gateway 在跑；本脚本只订阅话题，不打开串口"
        )

    def _time_sync_cb(self, msg: TimeSyncResponse) -> None:
        self._time_sync_any += 1
        if msg.cmd_echo == CMD_PING and msg.rtt_ms > 0:
            self._ping_count += 1
            self._offsets.append(msg.offset_ms)
            self._rtts.append(msg.rtt_ms)
            if msg.rtt_ms > self._rtt_warn_ms:
                self._rtt_warn_count += 1
        elif msg.cmd_echo == 0 and msg.system_tick_ms > 0:
            self._query_count += 1

    def _v3_cb(self, _msg: V3Status) -> None:
        self._v3_count += 1

    def _tick(self) -> None:
        now = time.monotonic()
        elapsed = now - self._start

        if self._ping_count == 0 and elapsed >= 15.0 and not self._no_data_warned:
            self._no_data_warned = True
            self.get_logger().error(
                "15s 内未收到 PING（ping=0）。常见原因："
                "① 本终端先停了 gateway（Ctrl+C）；"
                "② 未 source install/setup.bash；"
                "③ gateway 未发布 /jetson_rs232/time_sync。"
                "请在另一终端保持: ros2 launch rs232_gateway ..."
            )

        if now >= self._deadline:
            self._print_summary()
            rclpy.shutdown()
            return

        if now - self._last_report >= 60.0:
            elapsed = now - self._start
            v3_hz = self._v3_count / elapsed if elapsed > 0 else 0.0
            off = self._offsets[-1] if self._offsets else 0.0
            self.get_logger().info(
                f"[{elapsed:.0f}s] ping={self._ping_count} query={self._query_count} "
                f"v3≈{v3_hz:.1f}Hz offset={off:.1f} "
                f"rtt_warn={self._rtt_warn_count}"
            )
            self._last_report = now

    def _print_summary(self) -> None:
        elapsed = time.monotonic() - self._start
        v3_hz = self._v3_count / elapsed if elapsed > 0 else 0.0
        warn_pct = (
            100.0 * self._rtt_warn_count / self._ping_count if self._ping_count else 0.0
        )

        print("\n========== TimeSync Soak 汇总 ==========")
        print(f"时长: {elapsed:.1f}s")
        print(f"PING 次数: {self._ping_count}")
        print(f"QUERY 次数: {self._query_count}")
        print(f"V3 上行约: {v3_hz:.1f} Hz (总 {self._v3_count} 帧)")
        if self._offsets:
            print(
                f"offset_ms: min={min(self._offsets):.2f} max={max(self._offsets):.2f} "
                f"last={self._offsets[-1]:.2f} "
                f"span={max(self._offsets)-min(self._offsets):.2f} "
                f"stdev={statistics.pstdev(self._offsets):.3f}"
            )
        if self._rtts:
            p95_idx = max(0, int(len(self._rtts) * 0.95) - 1)
            print(
                f"RTT ms: min={min(self._rtts):.2f} max={max(self._rtts):.2f} "
                f"mean={statistics.mean(self._rtts):.2f} "
                f"p95={sorted(self._rtts)[p95_idx]:.2f}"
            )
        if self._ping_count == 0:
            print(
                "提示: PING=0 通常表示 soak 期间 gateway 未运行，"
                "本次结果不能用于长稳验收。"
            )
        print(
            f"RTT>{self._rtt_warn_ms}ms: {self._rtt_warn_count}/{self._ping_count} "
            f"({warn_pct:.1f}%)"
        )

        ok_offset = (
            len(self._offsets) > 0
            and (max(self._offsets) - min(self._offsets)) < 50.0
        )
        ok_warn = warn_pct < 5.0
        ok_v3 = v3_hz > 15.0
        print("验收:")
        print(f"  offset 漂移 <50ms: {'PASS' if ok_offset else 'CHECK'}")
        print(f"  RTT WARN <5%:       {'PASS' if ok_warn else 'CHECK'}")
        print(f"  V3 >15Hz:           {'PASS' if ok_v3 else 'CHECK'}")
        print("========================================\n")


def main() -> int:
    ap = argparse.ArgumentParser(description="Jetson TimeSync soak 监测")
    ap.add_argument("--duration", type=float, default=3600.0, help="监测时长(秒)")
    ap.add_argument("--rtt-warn-ms", type=float, default=RTT_WARN_MS)
    args = ap.parse_args()

    rclpy.init()
    node = TimeSoakNode(args.duration, args.rtt_warn_ms)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._print_summary()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
