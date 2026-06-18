#!/usr/bin/env python3
"""Jetson <-> F407 MCU Ethernet BLOB v2 UDP link test."""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

BLOB_MAGIC = 0xAB
BLOB_VER = 0x01
BLOB_HDR_LEN = 9

MSG_AGV_CONTROL = 0x01
MSG_AGV_MOTION = 0x02
MSG_MCU_STATUS = 0x03
MSG_SENSOR_BLOB = 0x04
MSG_GPS_COMPACT = 0x05

PAYLOAD_LEN: dict[int, int] = {
    MSG_AGV_CONTROL: 14,
    MSG_AGV_MOTION: 40,
    MSG_MCU_STATUS: 42,
    MSG_SENSOR_BLOB: 28,
    MSG_GPS_COMPACT: 32,
}

SAFETY_NAMES = {
    0x01: "NORMAL",
    0x02: "SPEED_LIMIT",
    0x03: "DEGRADED",
    0x04: "EMERGENCY",
}

DEFAULT_MCU_IP = "192.168.10.30"
DEFAULT_MCU_PORT = 50001
DEFAULT_BIND_IP = "0.0.0.0"
DEFAULT_LOCAL_PORT = 50002


def parse_blob_datagram(data: bytes) -> tuple[int, int, bytes] | None:
    if len(data) < BLOB_HDR_LEN or data[0] != BLOB_MAGIC:
        return None
    ver, msg_id, seq = data[1], data[2], data[3]
    plen = (data[4] << 8) | data[5]
    frag_idx, frag_cnt, flags = data[6], data[7], data[8]
    if ver != BLOB_VER or frag_idx != 0 or frag_cnt != 1 or flags != 0:
        return None
    expected = PAYLOAD_LEN.get(msg_id)
    if expected is not None and plen != expected:
        return None
    if len(data) != BLOB_HDR_LEN + plen:
        return None
    return msg_id, seq, data[BLOB_HDR_LEN : BLOB_HDR_LEN + plen]


def build_control_blob(seq: int, v_mm_s: int = 0, control_mode: int = 0x01) -> bytes:
    ts = int(time.time() * 1000) & 0xFFFFFFFF
    payload = struct.pack(">IhhhBBBB", ts, v_mm_s, 0, 0, control_mode, 0, 0, 0)
    hdr = struct.pack(">BBBBHBBB", BLOB_MAGIC, BLOB_VER, MSG_AGV_CONTROL, seq & 0xFF, 14, 0, 1, 0)
    return hdr + payload


def format_motion(payload: bytes) -> str:
    if len(payload) != PAYLOAD_LEN[MSG_AGV_MOTION]:
        return f"len={len(payload)}"
    v = struct.unpack_from(">h", payload, 13)[0]
    bat = struct.unpack_from(">H", payload, 11)[0]
    return f"v={v}mm/s bat={bat * 0.1:.1f}V"


def format_mcu_status(payload: bytes) -> str:
    if len(payload) != PAYLOAD_LEN[MSG_MCU_STATUS]:
        return f"len={len(payload)}"
    mseq, safety = payload[4], payload[5]
    limit = payload[7]
    jetson_seq = payload[40]
    name = SAFETY_NAMES.get(safety, f"0x{safety:02X}")
    return f"seq={mseq} safety={name} limit={limit}% jetson_seq={jetson_seq}"


def format_gps(payload: bytes) -> str:
    if len(payload) != PAYLOAD_LEN[MSG_GPS_COMPACT]:
        return f"len={len(payload)}"
    flags, num_sv = payload[4], payload[5]
    lat = struct.unpack_from(">i", payload, 10)[0]
    lon = struct.unpack_from(">i", payload, 14)[0]
    return f"flags=0x{flags:02X} sv={num_sv} lat={lat} lon={lon}"


def open_rx(bind_ip: str, local_port: int) -> socket.socket:
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind((bind_ip, local_port))
    rx.setblocking(False)
    return rx


def drain_rx(
    rx: socket.socket,
    counts: dict[int, int],
    *,
    show_hex: bool,
    verbose: bool,
    last_print: list[float],
) -> None:
    while True:
        try:
            data, addr = rx.recvfrom(4096)
        except BlockingIOError:
            break
        counts["_bytes"] = counts.get("_bytes", 0) + len(data)
        counts["_pkts"] = counts.get("_pkts", 0) + 1

        if show_hex:
            print(f"  RX {len(data)}B from {addr[0]}:{addr[1]}: {data.hex(' ')}")

        parsed = parse_blob_datagram(data)
        if parsed is None:
            counts["_bad"] = counts.get("_bad", 0) + 1
            if verbose:
                print(f"  drop {len(data)}B (not valid BLOB wire)")
            continue

        msg_id, seq, payload = parsed
        counts[msg_id] = counts.get(msg_id, 0) + 1

        now = time.monotonic()
        if not verbose and now - last_print[0] < 0.5:
            continue
        last_print[0] = now

        if msg_id == MSG_AGV_MOTION:
            print(f"  RX 0x02 seq={seq} {format_motion(payload)}")
        elif msg_id == MSG_MCU_STATUS:
            print(f"  RX 0x03 seq={seq} {format_mcu_status(payload)}")
        elif msg_id == MSG_GPS_COMPACT:
            print(f"  RX 0x05 seq={seq} {format_gps(payload)}")
        elif msg_id == MSG_SENSOR_BLOB:
            print(f"  RX 0x04 seq={seq} sensor_blob")
        else:
            print(f"  RX MSG 0x{msg_id:02X} seq={seq} plen={len(payload)}")


def run_listen_only(
    bind_ip: str,
    local_port: int,
    duration_s: float,
    *,
    show_hex: bool,
    verbose: bool,
) -> None:
    print(f"[A] listen-only {duration_s}s on {bind_ip}:{local_port}")
    rx = open_rx(bind_ip, local_port)
    counts: dict[int, int] = {}
    last_print = [0.0]
    deadline = time.monotonic() + duration_s
    try:
        while time.monotonic() < deadline:
            drain_rx(rx, counts, show_hex=show_hex, verbose=verbose, last_print=last_print)
            time.sleep(0.005)
    finally:
        rx.close()

    m02 = counts.get(MSG_AGV_MOTION, 0)
    m03 = counts.get(MSG_MCU_STATUS, 0)
    print(
        f"\nstats: pkts={counts.get('_pkts', 0)} bytes={counts.get('_bytes', 0)} "
        f"bad={counts.get('_bad', 0)} | 0x02={m02} 0x03={m03} 0x05={counts.get(MSG_GPS_COMPACT, 0)}"
    )
    if m02 == 0 and m03 == 0:
        print("FAIL: no BLOB uplink")
    else:
        print("OK: MCU->Jetson Ethernet uplink")


def run_heartbeat(
    bind_ip: str,
    local_port: int,
    mcu_ip: str,
    mcu_port: int,
    duration_s: float,
    rate_hz: float,
    v_mm_s: int,
    *,
    show_hex: bool,
    verbose: bool,
) -> None:
    period = 1.0 / rate_hz
    print(
        f"[B] TX 0x01 @ {rate_hz:.0f}Hz -> {mcu_ip}:{mcu_port}, "
        f"RX on {bind_ip}:{local_port}, {duration_s}s"
    )
    rx = open_rx(bind_ip, local_port)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    counts: dict[int, int] = {}
    last_print = [0.0]
    seq = 0
    tx_count = 0
    deadline = time.monotonic() + duration_s
    next_tx = time.monotonic()

    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_tx:
                frame = build_control_blob(seq, v_mm_s=v_mm_s)
                tx.sendto(frame, (mcu_ip, mcu_port))
                tx_count += 1
                if show_hex:
                    print(f"  TX 0x01 seq={seq}: {frame.hex(' ')}")
                seq = (seq + 1) & 0xFF
                next_tx += period
                if next_tx < now:
                    next_tx = now

            drain_rx(rx, counts, show_hex=show_hex, verbose=verbose, last_print=last_print)
            time.sleep(0.002)
    finally:
        rx.close()
        tx.close()

    m02 = counts.get(MSG_AGV_MOTION, 0)
    m03 = counts.get(MSG_MCU_STATUS, 0)
    print(
        f"\nstats: TX 0x01={tx_count} | RX 0x02={m02} 0x03={m03} "
        f"pkts={counts.get('_pkts', 0)} bad={counts.get('_bad', 0)}"
    )
    if m02 == 0 and m03 == 0:
        print("FAIL: heartbeat sent but no uplink")
    elif tx_count > 0:
        print("OK: bidirectional UDP BLOB; check 0x03 safety=NORMAL, jetson_seq matches")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Jetson <-> F407 Ethernet BLOB v2 UDP test",
        epilog=(
            "Examples:\n"
            "  python3 tools/jetson_eth_blob_test.py --listen-only --hex --time 15\n"
            "  python3 tools/jetson_eth_blob_test.py --bind-ip 192.168.10.201 --time 30"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--bind-ip", default=DEFAULT_BIND_IP)
    ap.add_argument("--local-port", type=int, default=DEFAULT_LOCAL_PORT)
    ap.add_argument("--mcu-ip", default=DEFAULT_MCU_IP)
    ap.add_argument("--mcu-port", type=int, default=DEFAULT_MCU_PORT)
    ap.add_argument("--time", type=float, default=15.0)
    ap.add_argument("--listen-only", action="store_true")
    ap.add_argument("--hex", action="store_true", help="print raw hex per datagram")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--rate-hz", type=float, default=50.0)
    ap.add_argument("--v-mm-s", type=int, default=0)
    args = ap.parse_args()

    print("=" * 60)
    print("Jetson <-> F407 Ethernet BLOB v2 (UDP)")
    print(f"  MCU downlink:  {args.mcu_ip}:{args.mcu_port}")
    print(f"  Jetson uplink: {args.bind_ip}:{args.local_port}")
    print(f"  mode: {'listen-only' if args.listen_only else f'bidir {args.rate_hz}Hz'}")
    print("=" * 60)

    try:
        if args.listen_only:
            run_listen_only(
                args.bind_ip,
                args.local_port,
                args.time,
                show_hex=args.hex,
                verbose=args.verbose,
            )
        else:
            run_heartbeat(
                args.bind_ip,
                args.local_port,
                args.mcu_ip,
                args.mcu_port,
                args.time,
                args.rate_hz,
                args.v_mm_s,
                show_hex=args.hex,
                verbose=args.verbose,
            )
    except KeyboardInterrupt:
        print("\nstopped")
    except OSError as exc:
        print(f"\nnetwork error: {exc}", file=sys.stderr)
        print("  check enp1s0f1=192.168.10.201/24, ping 192.168.10.30")
        sys.exit(1)


if __name__ == "__main__":
    main()
