"""用 TimeSync offset / QUERY 锚点给观测打 UTC 时间戳。"""

from __future__ import annotations

import time

from builtin_interfaces.msg import Time


def mono_ms() -> float:
    return time.monotonic() * 1000.0


def utc_sec_to_stamp(utc_sec: float) -> Time:
    if utc_sec < 0:
        utc_sec = 0.0
    sec = int(utc_sec)
    nanosec = int(round((utc_sec - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    stamp = Time()
    stamp.sec = sec
    stamp.nanosec = nanosec
    return stamp


def estimate_mcu_tick_ms(jetson_mono_ms: float, offset_ms: float) -> float:
    """t_mcu_ms ≈ jetson_mono_ms + offset_ms（offset = mcu - jetson）"""
    return jetson_mono_ms + offset_ms


def estimate_utc_sec(
    jetson_mono_ms: float,
    offset_ms: float,
    utc_anchor_sec: int,
    query_tick_ms: int,
) -> float | None:
    """utc ≈ utc_unix_sec + (t_mcu_ms - query_tick_ms) / 1000"""
    if utc_anchor_sec <= 0 or query_tick_ms <= 0:
        return None
    t_mcu_ms = estimate_mcu_tick_ms(jetson_mono_ms, offset_ms)
    return utc_anchor_sec + (t_mcu_ms - query_tick_ms) / 1000.0
