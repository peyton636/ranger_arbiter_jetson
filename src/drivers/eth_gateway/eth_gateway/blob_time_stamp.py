"""BLOB uplink stamps: mcu_timestamp_ms + offset -> Jetson/ROS time.

Phase2: replace with MCU TX_TICK_MS when available in payload.
"""

from __future__ import annotations

import time

from builtin_interfaces.msg import Time
from rclpy.clock import Clock
from rclpy.duration import Duration


def mono_ms() -> float:
    return time.monotonic() * 1000.0


def mcu_to_jetson_mono_ms(mcu_tick_ms: int | float, offset_ms: float) -> float:
    """t_jetson_mono = mcu_timestamp_ms - offset_ms"""
    return float(mcu_tick_ms) - offset_ms


def mcu_virtual_ms(offset_ms: float) -> float:
    """当前时刻的 MCU SYSTEM_TICK_MS 虚拟刻度：mono + offset。"""
    return mono_ms() + offset_ms


def mcu_virtual_to_ros_stamp(
    offset_ms: float,
    time_sync_valid: bool,
    clock: Clock,
    *,
    fallback: Time | None = None,
) -> Time:
    """把当前虚拟 MCU 时刻映射到 ROS stamp（与 BLOB 事件戳同一路径）。"""
    if time_sync_valid:
        return mcu_tick_to_ros_stamp(
            int(round(mcu_virtual_ms(offset_ms))),
            offset_ms,
            True,
            clock,
        )
    if fallback is not None:
        return fallback
    return clock.now().to_msg()


def time_sync_response_ros_stamp(
    clock: Clock,
    offset_ms: float,
    offset_valid: bool,
    mcu_tick_ms: int = 0,
) -> Time:
    """TimeSync 0x108 响应 / topic 用：优先 MCU tick，否则当前虚拟 MCU 时刻。"""
    if offset_valid and mcu_tick_ms > 0:
        return mcu_tick_to_ros_stamp(mcu_tick_ms, offset_ms, True, clock)
    return mcu_virtual_to_ros_stamp(offset_ms, offset_valid, clock)


def jetson_mono_to_ros_stamp(jetson_mono_ms: float, clock: Clock) -> Time:
    delta_ns = int(round((jetson_mono_ms - mono_ms()) * 1e6))
    return (clock.now() + Duration(nanoseconds=delta_ns)).to_msg()


def mcu_tick_to_ros_stamp(
    mcu_tick_ms: int,
    offset_ms: float,
    time_sync_valid: bool,
    clock: Clock,
    *,
    fallback: Time | None = None,
) -> Time:
    if time_sync_valid and mcu_tick_ms > 0:
        return jetson_mono_to_ros_stamp(
            mcu_to_jetson_mono_ms(mcu_tick_ms, offset_ms), clock
        )
    if fallback is not None:
        return fallback
    return clock.now().to_msg()
