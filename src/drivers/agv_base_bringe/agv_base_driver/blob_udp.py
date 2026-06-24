"""Parse BLOB wire frames and 0xA5 service frames from UDP datagrams."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

BLOB_MAGIC = 0xAB
BLOB_VER = 0x01
BLOB_HDR_LEN = 9

SERVICE_MAGIC = 0xA5
SERVICE_LEN = 11

EthRxItem = (
    tuple[Literal["blob"], int, int, bytes]
    | tuple[Literal["svc"], int, bytes, float]
)


@dataclass
class EthRxStats:
    bytes_in: int = 0
    pkts_in: int = 0
    bad_pkts: int = 0
    blob_02: int = 0
    blob_03: int = 0
    blob_04: int = 0
    blob_05: int = 0
    blob_other: int = 0
    svc_rx: int = 0

    def snapshot(self) -> dict[str, int]:
        return {
            "bytes_in": self.bytes_in,
            "pkts_in": self.pkts_in,
            "bad_pkts": self.bad_pkts,
            "blob_02": self.blob_02,
            "blob_03": self.blob_03,
            "blob_04": self.blob_04,
            "blob_05": self.blob_05,
            "blob_other": self.blob_other,
            "svc_rx": self.svc_rx,
        }


@dataclass
class EthBlobParser:
    stats: EthRxStats = field(default_factory=EthRxStats)

    def parse_datagram(self, data: bytes) -> EthRxItem | None:
        self.stats.bytes_in += len(data)
        self.stats.pkts_in += 1
        if not data:
            self.stats.bad_pkts += 1
            return None

        if data[0] == SERVICE_MAGIC:
            return self._parse_service(data)
        if data[0] == BLOB_MAGIC:
            return self._parse_blob(data)
        self.stats.bad_pkts += 1
        return None

    def _parse_service(self, data: bytes) -> EthRxItem | None:
        if len(data) != SERVICE_LEN:
            self.stats.bad_pkts += 1
            return None
        can_id = (data[1] << 8) | data[2]
        self.stats.svc_rx += 1
        return ("svc", can_id, data[3:11])

    def _parse_blob(self, data: bytes) -> EthRxItem | None:
        if len(data) < BLOB_HDR_LEN:
            self.stats.bad_pkts += 1
            return None
        ver, msg_id, seq = data[1], data[2], data[3]
        plen = (data[4] << 8) | data[5]
        frag_idx, frag_cnt, flags = data[6], data[7], data[8]
        if ver != BLOB_VER or frag_idx != 0 or frag_cnt != 1 or flags != 0:
            self.stats.bad_pkts += 1
            return None
        if len(data) != BLOB_HDR_LEN + plen:
            self.stats.bad_pkts += 1
            return None
        if msg_id == 0x02:
            self.stats.blob_02 += 1
        elif msg_id == 0x03:
            self.stats.blob_03 += 1
        elif msg_id == 0x04:
            self.stats.blob_04 += 1
        elif msg_id == 0x05:
            self.stats.blob_05 += 1
        else:
            self.stats.blob_other += 1
        return ("blob", msg_id, seq, data[BLOB_HDR_LEN : BLOB_HDR_LEN + plen])
