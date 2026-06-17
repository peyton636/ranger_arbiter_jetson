"""Rs232StreamParser 混流单元测试。"""

from __future__ import annotations

import struct
import unittest

from rs232_gateway.blob_codec import BLOB_HDR_LEN, encode_agv_control
from rs232_gateway.service_frame import (
    Rs232StreamParser,
    SERVICE_MAGIC,
    encode_service_request,
    validate_rs232_blob_header,
)


def _blob_frame(msg_id: int, payload: bytes, seq: int = 1) -> bytes:
    hdr = bytearray(BLOB_HDR_LEN)
    hdr[0] = 0xAB
    hdr[1] = 0x01
    hdr[2] = msg_id
    hdr[3] = seq & 0xFF
    plen = len(payload)
    hdr[4] = (plen >> 8) & 0xFF
    hdr[5] = plen & 0xFF
    hdr[6] = 0
    hdr[7] = 1
    hdr[8] = 0
    return bytes(hdr) + payload


def _mcu_status_payload(seq: int = 7, safety: int = 1) -> bytes:
    body = bytearray(42)
    body[4] = seq & 0xFF
    body[5] = safety
    body[6] = 0
    body[7] = 100
    return bytes(body)


def _motion_payload() -> bytes:
    body = bytearray(40)
    struct.pack_into(">h", body, 13, 96)
    struct.pack_into(">H", body, 11, 504)
    return bytes(body)


class TestRs232StreamParser(unittest.TestCase):
    def test_validate_rs232_header_rejects_bad_frag(self) -> None:
        hdr = bytearray(BLOB_HDR_LEN)
        hdr[0] = 0xAB
        hdr[1] = 0x01
        hdr[2] = 0x03
        hdr[4] = 0
        hdr[5] = 42
        hdr[6] = 0
        hdr[7] = 2
        hdr[8] = 0
        self.assertIsNone(validate_rs232_blob_header(bytes(hdr)))

    def test_rejects_oversize_len_without_blocking(self) -> None:
        parser = Rs232StreamParser(parse_v3=False)
        bad = bytes([0xAB, 0x01, 0x03, 0x01, 0x10, 0x00, 0x00, 0x01, 0x00])
        good = _blob_frame(0x03, _mcu_status_payload())
        items = list(parser.feed(bad + good))
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0][0], "blob")
        self.assertEqual(items[0][1], 0x03)
        self.assertGreater(parser.stats.hdr_reject, 0)

    def test_mixed_blob_svc_blob(self) -> None:
        parser = Rs232StreamParser(parse_v3=False)
        motion = _blob_frame(0x02, _motion_payload(), seq=2)
        svc = encode_service_request(0x108, bytes([0x03, 0x01]) + bytes(6))
        mcu = _blob_frame(0x03, _mcu_status_payload(safety=1), seq=3)
        downlink = encode_agv_control(1, 12345, control_mode=1)
        stream = motion + svc + mcu + downlink

        items = list(parser.feed(stream))
        blob_ids = [it[1] for it in items if it[0] == "blob"]
        svc_count = sum(1 for it in items if it[0] == "svc")

        self.assertEqual(blob_ids[:2], [0x02, 0x03])
        self.assertEqual(svc_count, 1)
        self.assertEqual(svc[0], SERVICE_MAGIC)
        self.assertEqual(parser.stats.blob_02, 1)
        self.assertEqual(parser.stats.blob_03, 1)
        self.assertEqual(parser.stats.svc_rx, 1)

    def test_downlink_0x01_ignored_but_no_stall(self) -> None:
        parser = Rs232StreamParser(parse_v3=False)
        down = encode_agv_control(5, 999, control_mode=1)
        up = _blob_frame(0x03, _mcu_status_payload(), seq=9)
        items = list(parser.feed(down + up))
        self.assertEqual([it[1] for it in items if it[0] == "blob"], [0x01, 0x03])


if __name__ == "__main__":
    unittest.main()
