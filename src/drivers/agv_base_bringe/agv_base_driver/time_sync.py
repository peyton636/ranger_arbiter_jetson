"""Jetson ↔ STM32 时间同步：START / PING / STOP / QUERY + offset / RTT。"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional

from agv_base_driver.service_frame import CAN_ID_TIME_SYNC_REQ, encode_service_request

CMD_QUERY = 0x01
CMD_START = 0x02
CMD_PING = 0x03
CMD_STOP = 0x04


def mono_ms() -> float:
    return time.monotonic() * 1000.0


def u32be(data: bytes, index: int) -> int:
    return (
        (data[index] << 24)
        | (data[index + 1] << 16)
        | (data[index + 2] << 8)
        | data[index + 3]
    )


def pack_u32be(value: int) -> bytes:
    value &= 0xFFFFFFFF
    return bytes(
        [
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ]
    )


def build_query_payload() -> bytes:
    return bytes([CMD_QUERY, 0, 0, 0, 0, 0, 0, 0])


def build_start_payload(session_id: int, jetson_mono_ms: int) -> bytes:
    return bytes([CMD_START, session_id & 0xFF]) + pack_u32be(jetson_mono_ms) + bytes(2)


def build_ping_payload(seq: int, t1_ms: int) -> bytes:
    return bytes([CMD_PING, seq & 0xFF]) + pack_u32be(t1_ms) + bytes(2)


def build_stop_payload(session_id: int) -> bytes:
    return bytes([CMD_STOP, session_id & 0xFF, 0, 0, 0, 0, 0, 0])


def parse_query_response(payload: bytes) -> dict:
    return {
        "format": "query",
        "system_tick_ms": u32be(payload, 0),
        "utc_unix_sec": u32be(payload, 4),
    }


def parse_ping_response(payload: bytes) -> dict:
    return {
        "format": "ping",
        "cmd_echo": payload[0],
        "seq_echo": payload[1],
        "mcu_tick_rx": u32be(payload, 2),
        "gps_utc_valid": bool(payload[6] & 1),
        "proc_ms": payload[7] * 0.1,
    }


@dataclass
class PendingRequest:
    cmd: int
    t1_ms: float
    echo: int


@dataclass
class TimeSyncUpdate:
    format: str
    system_tick_ms: int = 0
    utc_unix_sec: int = 0
    cmd_echo: int = 0
    seq_echo: int = 0
    mcu_tick_rx: int = 0
    gps_utc_valid: bool = False
    proc_ms: float = 0.0
    rtt_ms: float = 0.0
    rtt_ema_ms: float = 0.0
    offset_ms: float = 0.0
    offset_sample_ms: float = 0.0
    offset_valid: bool = False
    sample_accepted: bool = False
    rtt_warn: bool = False


@dataclass
class JetsonTimeSync:
    """维护 offset_ms / rtt_ms，生成 0x107 请求帧。"""

    session_id: int = 1
    ping_burst_count: int = 10
    ping_burst_interval_s: float = 0.1
    ping_interval_s: float = 1.0
    query_interval_s: float = 10.0
    rtt_warn_ms: float = 50.0
    offset_ema_alpha: float = 0.2

    offset_ms: float = 0.0
    rtt_ms: float = 0.0
    rtt_ema_ms: float = 0.0
    offset_valid: bool = False
    session_active: bool = False
    ping_seq: int = 0

    _burst_remaining: int = 0
    _next_burst_mono: float = 0.0
    _last_ping_mono: float = 0.0
    _last_query_mono: float = 0.0
    _pending_ping: Deque[PendingRequest] = field(
        default_factory=lambda: deque(maxlen=32)
    )
    _pending_query: Deque[PendingRequest] = field(
        default_factory=lambda: deque(maxlen=8)
    )

    def mcu_to_jetson(self, mcu_tick_ms: int | float) -> float:
        return float(mcu_tick_ms) - self.offset_ms

    def jetson_to_mcu(self, jetson_mono_ms: float) -> float:
        return jetson_mono_ms + self.offset_ms

    def mcu_virtual_ms(self) -> float:
        """与 MCU SYSTEM_TICK_MS 同刻度的虚拟时刻（mono + offset）。"""
        return mono_ms() + self.offset_ms

    def begin_session(self) -> bytes:
        t1 = mono_ms()
        payload = build_start_payload(self.session_id, int(t1))
        self._pending_ping.append(PendingRequest(CMD_START, t1, self.session_id))
        self.session_active = True
        self._burst_remaining = self.ping_burst_count
        now = time.monotonic()
        self._next_burst_mono = now + self.ping_burst_interval_s
        self._last_ping_mono = now
        self._last_query_mono = now
        return encode_service_request(CAN_ID_TIME_SYNC_REQ, payload)

    def stop_session(self) -> bytes:
        self.session_active = False
        self._burst_remaining = 0
        self._pending_ping.clear()
        self._pending_query.clear()
        return encode_service_request(
            CAN_ID_TIME_SYNC_REQ, build_stop_payload(self.session_id)
        )

    def build_ping(self) -> bytes:
        t1 = mono_ms()
        self.ping_seq = (self.ping_seq + 1) & 0xFF
        self._pending_ping.append(PendingRequest(CMD_PING, t1, self.ping_seq))
        return encode_service_request(
            CAN_ID_TIME_SYNC_REQ, build_ping_payload(self.ping_seq, int(t1))
        )

    def build_query(self) -> bytes:
        self._pending_query.append(PendingRequest(CMD_QUERY, 0.0, 0))
        return encode_service_request(CAN_ID_TIME_SYNC_REQ, build_query_payload())

    def tick(self, now_mono: Optional[float] = None) -> List[bytes]:
        if not self.session_active:
            return []
        now = now_mono if now_mono is not None else time.monotonic()
        frames: List[bytes] = []

        if self._burst_remaining > 0 and now >= self._next_burst_mono:
            frames.append(self.build_ping())
            self._burst_remaining -= 1
            self._next_burst_mono = now + self.ping_burst_interval_s
            self._last_ping_mono = now

        if now - self._last_ping_mono >= self.ping_interval_s:
            frames.append(self.build_ping())
            self._last_ping_mono = now

        if now - self._last_query_mono >= self.query_interval_s:
            frames.append(self.build_query())
            self._last_query_mono = now

        return frames

    def _pop_ping_pending(self, payload: bytes) -> Optional[PendingRequest]:
        if payload[0] not in (CMD_START, CMD_PING):
            return None
        echo = payload[1]
        for idx, req in enumerate(self._pending_ping):
            if req.cmd == payload[0] and req.echo == echo:
                del self._pending_ping[idx]
                return req
        # 乱序/丢包：PING 按 seq 对不上时，取最早一条 PING pending（不用 START）
        if payload[0] == CMD_PING:
            for idx, req in enumerate(self._pending_ping):
                if req.cmd == CMD_PING:
                    del self._pending_ping[idx]
                    return req
        return None

    def _pop_query_pending(self) -> Optional[PendingRequest]:
        if not self._pending_query:
            return None
        return self._pending_query.popleft()

    def _is_bad_rtt(self, rtt: float) -> bool:
        """Reject sample from offset EMA (absolute threshold only)."""
        return rtt <= 0.0 or rtt > self.rtt_warn_ms

    def _rtt_spike_warn(self, rtt: float) -> bool:
        """Log-only spike detector; does not discard the sample."""
        if rtt > self.rtt_warn_ms:
            return True
        if self.rtt_ema_ms > 5.0 and rtt > self.rtt_ema_ms * 2.5:
            return True
        return False

    def on_response(
        self, payload: bytes, t4_ms: Optional[float] = None
    ) -> Optional[TimeSyncUpdate]:
        if len(payload) < 8:
            return None

        if payload[0] in (CMD_START, CMD_PING):
            return self._on_ping_response(payload, t4_ms)

        req = self._pop_query_pending()
        parsed = parse_query_response(payload)
        return TimeSyncUpdate(
            format="query",
            system_tick_ms=parsed["system_tick_ms"],
            utc_unix_sec=parsed["utc_unix_sec"],
            offset_ms=self.offset_ms,
            rtt_ms=self.rtt_ms,
            rtt_ema_ms=self.rtt_ema_ms,
            offset_valid=self.offset_valid,
        )

    def _on_ping_response(
        self, payload: bytes, t4_ms: Optional[float]
    ) -> Optional[TimeSyncUpdate]:
        parsed = parse_ping_response(payload)
        req = self._pop_ping_pending(payload)
        t1 = req.t1_ms if req is not None else mono_ms()
        t4 = t4_ms if t4_ms is not None else mono_ms()
        rtt = max(0.0, t4 - t1)
        proc = parsed["proc_ms"]
        one_way = max(0.0, (rtt - proc) / 2.0)
        offset_sample = parsed["mcu_tick_rx"] - t1 - one_way
        rtt_warn = self._rtt_spike_warn(rtt)
        sample_accepted = req is not None and not self._is_bad_rtt(rtt)

        if sample_accepted:
            alpha = self.offset_ema_alpha
            jump_ms = abs(offset_sample - self.offset_ms)
            if not self.offset_valid or jump_ms > 60000.0:
                # 首次对表或 MCU 重启 tick 归零：直接采纳，避免 EMA 慢慢爬
                self.offset_ms = offset_sample
            else:
                self.offset_ms = (1.0 - alpha) * self.offset_ms + alpha * offset_sample
            self.offset_valid = True
            self.rtt_ms = rtt
            if self.rtt_ema_ms <= 0.0:
                self.rtt_ema_ms = rtt
            else:
                self.rtt_ema_ms = 0.8 * self.rtt_ema_ms + 0.2 * rtt

        return TimeSyncUpdate(
            format="ping",
            system_tick_ms=parsed["mcu_tick_rx"],
            utc_unix_sec=0,
            cmd_echo=parsed["cmd_echo"],
            seq_echo=parsed["seq_echo"],
            mcu_tick_rx=parsed["mcu_tick_rx"],
            gps_utc_valid=parsed["gps_utc_valid"],
            proc_ms=proc,
            rtt_ms=rtt,
            rtt_ema_ms=self.rtt_ema_ms,
            offset_ms=self.offset_ms,
            offset_sample_ms=offset_sample,
            offset_valid=self.offset_valid,
            sample_accepted=sample_accepted,
            rtt_warn=rtt_warn,
        )
