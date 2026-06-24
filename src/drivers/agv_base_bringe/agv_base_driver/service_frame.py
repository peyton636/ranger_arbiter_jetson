"""RS232 混传帧解析：V3 0xAA(24B) + BLOB 0xAB(9+LEN) + 服务帧 0xA5(11B)。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator, Literal, Union

from agv_base_driver.jetson_protocol import (
    FRAME_HEADER,
    FRAME_LEN,
    FRAME_TYPE_DOWN,
    FRAME_TYPE_UP_EXT,
    FRAME_TYPE_UP_STATUS,
    xor_frame,
)

from agv_base_driver.blob_codec import BLOB_HDR_LEN, BLOB_MAGIC, BLOB_VER, PAYLOAD_LEN

SERVICE_MAGIC = 0xA5
SERVICE_LEN = 11

CAN_ID_GPS_A = 0x104
CAN_ID_GPS_B = 0x105
CAN_ID_GPS_C = 0x106
CAN_ID_TIME_SYNC_REQ = 0x107
CAN_ID_TIME_SYNC_RSP = 0x108
CAN_ID_FAULT = 0x109
CAN_ID_STATUS_QUERY = 0x10A
CAN_ID_STATUS_SNAPSHOT = 0x10B

# MCU → Jetson BLOB 上行（MSG 0x01 为 Jetson 下行）
UPLINK_BLOB_MSG_IDS = frozenset(
    {
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
        0x07,
        0x08,
        0x0B,
        0x10,
    }
)

StreamItem = (
    tuple[Literal["v3"], bytes]
    | tuple[Literal["svc"], int, bytes, float]
    | tuple[Literal["blob"], int, int, bytes]
)


@dataclass
class RxStats:
    """RX 解析计数（与 F407 [JETSON RX] 对照）。"""

    blob_02: int = 0
    blob_03: int = 0
    blob_04: int = 0
    blob_other: int = 0
    svc_rx: int = 0
    v3_rx: int = 0
    hdr_reject: int = 0
    resync: int = 0
    bytes_in: int = 0

    def snapshot(self) -> dict[str, int]:
        return {
            "blob_02": self.blob_02,
            "blob_03": self.blob_03,
            "blob_04": self.blob_04,
            "blob_other": self.blob_other,
            "svc_rx": self.svc_rx,
            "v3_rx": self.v3_rx,
            "hdr_reject": self.hdr_reject,
            "resync": self.resync,
            "bytes_in": self.bytes_in,
        }


def validate_rs232_blob_header(hdr: bytes) -> tuple[int, int] | None:
    """RS232 单帧 BLOB 头校验（对齐 docs/Jetson_BLOB协议_v2.md）。"""
    if len(hdr) < BLOB_HDR_LEN:
        return None
    if hdr[0] != BLOB_MAGIC or hdr[1] != BLOB_VER:
        return None
    if hdr[6] != 0 or hdr[7] != 1 or hdr[8] != 0:
        return None
    msg_id = hdr[2]
    plen = (hdr[4] << 8) | hdr[5]
    expected = PAYLOAD_LEN.get(msg_id)
    if expected is None or expected != plen:
        return None
    return msg_id, plen


class Rs232StreamParser:
    """从 RS232 字节流提取 V3 / BLOB / 服务帧（混流安全，提前头校验）。"""

    def __init__(self, *, parse_v3: bool = True) -> None:
        self._buf = bytearray()
        self._parse_v3 = parse_v3
        self.stats = RxStats()

    def reset(self) -> None:
        self._buf.clear()
        self.stats = RxStats()

    def feed(self, data: bytes) -> Iterator[StreamItem]:
        if not data:
            return
        self.stats.bytes_in += len(data)
        self._buf.extend(data)
        yield from self._drain()

    def _drain(self) -> Iterator[StreamItem]:
        while self._buf:
            magic = self._buf[0]
            if magic == BLOB_MAGIC:
                item = self._take_blob()
                if item is None:
                    break
                if item is False:
                    continue
                yield item
            elif magic == SERVICE_MAGIC:
                item = self._take_svc()
                if item is None:
                    break
                yield item
            elif magic == FRAME_HEADER and self._parse_v3:
                item = self._take_v3()
                if item is None:
                    break
                if item is False:
                    continue
                yield item
            else:
                self._buf.pop(0)
                self.stats.resync += 1

    def _take_blob(self) -> Union[StreamItem, None, Literal[False]]:
        if len(self._buf) < BLOB_HDR_LEN:
            return None
        hdr = bytes(self._buf[:BLOB_HDR_LEN])
        validated = validate_rs232_blob_header(hdr)
        if validated is None:
            self._buf.pop(0)
            self.stats.hdr_reject += 1
            self.stats.resync += 1
            return False
        msg_id, plen = validated
        wire_len = BLOB_HDR_LEN + plen
        if len(self._buf) < wire_len:
            return None
        frame = bytes(self._buf[:wire_len])
        del self._buf[:wire_len]
        payload = frame[BLOB_HDR_LEN:]
        seq = frame[3]
        self._count_blob(msg_id)
        return ("blob", msg_id, seq, payload)

    def _take_svc(self) -> StreamItem | None:
        if len(self._buf) < SERVICE_LEN:
            return None
        frame = bytes(self._buf[:SERVICE_LEN])
        del self._buf[:SERVICE_LEN]
        can_id = (frame[1] << 8) | frame[2]
        self.stats.svc_rx += 1
        return ("svc", can_id, frame[3:11])

    def _take_v3(self) -> Union[StreamItem, None, Literal[False]]:
        if len(self._buf) < FRAME_LEN:
            return None
        frame = bytes(self._buf[:FRAME_LEN])
        if frame[1] not in (
            FRAME_TYPE_DOWN,
            FRAME_TYPE_UP_STATUS,
            FRAME_TYPE_UP_EXT,
        ):
            self._buf.pop(0)
            self.stats.hdr_reject += 1
            self.stats.resync += 1
            return False
        if xor_frame(frame) != frame[23]:
            self._buf.pop(0)
            self.stats.hdr_reject += 1
            self.stats.resync += 1
            return False
        del self._buf[:FRAME_LEN]
        self.stats.v3_rx += 1
        return ("v3", frame)

    def _count_blob(self, msg_id: int) -> None:
        if msg_id == 0x02:
            self.stats.blob_02 += 1
        elif msg_id == 0x03:
            self.stats.blob_03 += 1
        elif msg_id == 0x04:
            self.stats.blob_04 += 1
        else:
            self.stats.blob_other += 1


def encode_service_request(can_id: int, payload: bytes | None = None) -> bytes:
    """构造 11B 服务帧（Jetson → STM32，如 0x107/0x10A）。"""
    if payload is None:
        payload = bytes([0x01]) + bytes(7)
    elif len(payload) < 8:
        payload = payload + bytes(8 - len(payload))
    else:
        payload = payload[:8]
    return bytes(
        [
            SERVICE_MAGIC,
            (can_id >> 8) & 0xFF,
            can_id & 0xFF,
            *payload,
        ]
    )
