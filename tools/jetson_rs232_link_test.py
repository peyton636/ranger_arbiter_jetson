#!/usr/bin/env python3
"""
Jetson ↔ STM32B RS232 链路测试（步骤 A+B）

用法：
  # V3 模式（旧固件 JETSON_USE_BLOB_V2=0）
  python3 jetson_rs232_link_test.py --port /dev/ttyUSB6 --time 10

  # BLOB v2 模式（默认，MCU JETSON_USE_BLOB_V2=1）
  python3 jetson_rs232_link_test.py --port /dev/ttyUSB6 --blob-v2 --time 10

  # 步骤 A：只收不发
  python3 jetson_rs232_link_test.py --port /dev/ttyUSB6 --listen-only --blob-v2 --time 5
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("请先安装: sudo apt install python3-serial", file=sys.stderr)
    sys.exit(1)

FRAME_LEN = 24
FRAME_HEADER = 0xAA
FRAME_TYPE_DOWN = 0x01
FRAME_TYPE_UP_STATUS = 0x02
FRAME_TYPE_UP_EXT = 0x03

SAFETY_NAMES = {
    0x01: "NORMAL",
    0x02: "SPEED_LIMIT",
    0x03: "DEGRADED",
    0x04: "EMERGENCY",
}

BLOB_MAGIC = 0xAB
BLOB_HDR_LEN = 9
MSG_AGV_CONTROL = 0x01
MSG_AGV_MOTION = 0x02
MSG_MCU_STATUS = 0x03
MSG_SENSOR_BLOB = 0x04
BLOB_PAYLOAD_LEN = {
    MSG_AGV_CONTROL: 14,
    MSG_AGV_MOTION: 40,
    MSG_MCU_STATUS: 42,
    MSG_SENSOR_BLOB: 28,
}


def xor24(buf: bytes | bytearray) -> int:
    c = 0
    for b in buf[:23]:
        c ^= b
    return c


def make_downlink(seq: int, v: int = 0, omega: int = 0, steer: int = 0, mode_req: int = 1) -> bytes:
    f = bytearray(FRAME_LEN)
    f[0], f[1], f[2] = FRAME_HEADER, FRAME_TYPE_DOWN, seq & 0xFF
    f[3] = mode_req & 0xFF
    for val, off in ((v, 4), (omega, 6), (steer, 8)):
        if val < 0:
            val += 0x10000
        f[off] = (val >> 8) & 0xFF
        f[off + 1] = val & 0xFF
    f[10] = 0x00  # 阿克曼
    f[23] = xor24(f)
    return bytes(f)


def parse_frame(raw: bytes) -> dict | None:
    if len(raw) != FRAME_LEN or raw[0] != FRAME_HEADER:
        return None
    if xor24(raw) != raw[23]:
        return None
    ftype = raw[1]
    if ftype not in (FRAME_TYPE_DOWN, FRAME_TYPE_UP_STATUS, FRAME_TYPE_UP_EXT):
        return None
    info: dict = {
        "type": ftype,
        "seq": raw[2],
        "raw": raw,
    }
    if ftype == FRAME_TYPE_UP_STATUS:
        info["safety_state"] = raw[3]
        info["safety_name"] = SAFETY_NAMES.get(raw[3], f"UNKNOWN(0x{raw[3]:02X})")
        info["link_state"] = raw[4]
        info["limit_factor"] = raw[5]
        info["v_mm_s"] = int.from_bytes(raw[6:8], "big", signed=True)
        info["sonar_front"] = int.from_bytes(raw[12:14], "big")
    return info


class FrameParser:
    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[dict]:
        out: list[dict] = []
        self._buf.extend(data)
        while len(self._buf) >= FRAME_LEN:
            if self._buf[0] != FRAME_HEADER:
                self._buf.pop(0)
                continue
            if self._buf[1] not in (
                FRAME_TYPE_DOWN,
                FRAME_TYPE_UP_STATUS,
                FRAME_TYPE_UP_EXT,
            ):
                self._buf.pop(0)
                continue
            frame = bytes(self._buf[:FRAME_LEN])
            del self._buf[:FRAME_LEN]
            parsed = parse_frame(frame)
            if parsed:
                out.append(parsed)
        return out


def make_blob_downlink(seq: int, v: int = 0, omega: int = 0, steer: int = 0, mode_req: int = 1) -> bytes:
    ts = int(time.monotonic() * 1000) & 0xFFFFFFFF
    payload = struct.pack(">IhhhBBBB", ts, v, omega, steer, mode_req & 0xFF, 0, 0, 0)
    hdr = bytearray(BLOB_HDR_LEN)
    hdr[0] = BLOB_MAGIC
    hdr[1] = 0x01
    hdr[2] = MSG_AGV_CONTROL
    hdr[3] = seq & 0xFF
    hdr[4] = 0
    hdr[5] = len(payload)
    hdr[6] = 0
    hdr[7] = 1
    hdr[8] = 0
    return bytes(hdr) + payload


def parse_blob_motion(payload: bytes) -> dict | None:
    if len(payload) != BLOB_PAYLOAD_LEN[MSG_AGV_MOTION]:
        return None
    return {
        "type": MSG_AGV_MOTION,
        "v_mm_s": struct.unpack_from(">h", payload, 13)[0],
        "bat_v": struct.unpack_from(">H", payload, 11)[0],
    }


def parse_blob_mcu(payload: bytes) -> dict | None:
    if len(payload) != BLOB_PAYLOAD_LEN[MSG_MCU_STATUS]:
        return None
    safety = payload[5]
    return {
        "type": MSG_MCU_STATUS,
        "seq": payload[4],
        "safety_state": safety,
        "safety_name": SAFETY_NAMES.get(safety, f"UNKNOWN(0x{safety:02X})"),
        "link_state": payload[6],
        "limit_factor": payload[7],
        "sonar_front": struct.unpack_from(">H", payload, 14)[0],
    }


class BlobFrameParser:
    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[dict]:
        out: list[dict] = []
        self._buf.extend(data)
        while self._buf:
            if self._buf[0] != BLOB_MAGIC:
                self._buf.pop(0)
                continue
            if len(self._buf) < BLOB_HDR_LEN:
                break
            plen = (self._buf[4] << 8) | self._buf[5]
            wire_len = BLOB_HDR_LEN + plen
            if len(self._buf) < wire_len:
                break
            frame = bytes(self._buf[:wire_len])
            del self._buf[:wire_len]
            msg_id = frame[2]
            payload = frame[BLOB_HDR_LEN:]
            if BLOB_PAYLOAD_LEN.get(msg_id) != plen:
                continue
            if msg_id == MSG_AGV_MOTION:
                parsed = parse_blob_motion(payload)
            elif msg_id == MSG_MCU_STATUS:
                parsed = parse_blob_mcu(payload)
            elif msg_id == MSG_SENSOR_BLOB:
                parsed = {"type": MSG_SENSOR_BLOB}
            else:
                parsed = {"type": msg_id}
            if parsed:
                parsed["msg_id"] = msg_id
                out.append(parsed)
        return out


def run_listen_only(ser: serial.Serial, duration_s: float, blob_v2: bool) -> None:
    if blob_v2:
        print(f"[步骤 A] BLOB v2 只收不发 {duration_s}s，等待 MSG 0x02/0x03/0x04 …")
        parser = BlobFrameParser()
        counts: dict[int, int] = {}
    else:
        print(f"[步骤 A] 只收不发 {duration_s}s，等待 STM32 主动发 0x02/0x03 …")
        parser = FrameParser()
        counts = {FRAME_TYPE_UP_STATUS: 0, FRAME_TYPE_UP_EXT: 0}
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        n = ser.in_waiting
        if n > 0:
            for p in parser.feed(ser.read(n)):
                if blob_v2:
                    mid = p["type"]
                    counts[mid] = counts.get(mid, 0) + 1
                    if mid == MSG_MCU_STATUS:
                        print(
                            f"  RX 0x03 seq={p['seq']} "
                            f"仲裁={p['safety_name']} limit={p['limit_factor']}% "
                            f"sonarF={p['sonar_front']}"
                        )
                    elif mid == MSG_AGV_MOTION:
                        print(f"  RX 0x02 v={p['v_mm_s']}mm/s bat={p['bat_v'] * 0.1:.1f}V")
                    else:
                        print(f"  RX MSG 0x{mid:02X}")
                else:
                    counts[p["type"]] = counts.get(p["type"], 0) + 1
                    if p["type"] == FRAME_TYPE_UP_STATUS:
                        print(
                            f"  RX 0x02 seq={p['seq']} "
                            f"仲裁={p['safety_name']} limit={p['limit_factor']}% "
                            f"v={p['v_mm_s']}mm/s sonarF={p['sonar_front']}"
                        )
                    else:
                        print(f"  RX 0x03 seq={p['seq']} (扩展帧)")
        time.sleep(0.01)

    if blob_v2:
        m02 = counts.get(MSG_AGV_MOTION, 0)
        m03 = counts.get(MSG_MCU_STATUS, 0)
        print(f"\n统计: 0x02={m02}, 0x03={m03}, 0x04={counts.get(MSG_SENSOR_BLOB, 0)}")
        if m02 == 0 and m03 == 0:
            print("❌ 步骤 A 未通过：未收到 BLOB 上行。检查接线与 JETSON_USE_BLOB_V2=1。")
        else:
            print("✅ 步骤 A 通过：物理层有 BLOB 上行。")
    else:
        print(f"\n统计: 0x02={counts.get(FRAME_TYPE_UP_STATUS, 0)} 帧, "
              f"0x03={counts.get(FRAME_TYPE_UP_EXT, 0)} 帧")
        if counts.get(FRAME_TYPE_UP_STATUS, 0) == 0:
            print("❌ 步骤 A 未通过：未收到 0x02。检查接线、STM32 上电、PA2/PA3 是否 RS232 模式。")
        else:
            print("✅ 步骤 A 通过：物理层有上行。")


def run_heartbeat(ser: serial.Serial, duration_s: float, blob_v2: bool) -> None:
    if blob_v2:
        print(f"[步骤 B] BLOB v2: 20ms 发 MSG 0x01 心跳 + 读上行，持续 {duration_s}s …")
        parser = BlobFrameParser()
        make_tx = make_blob_downlink
    else:
        print(f"[步骤 B] 20ms 发 0x01 心跳(v=0) + 读上行，持续 {duration_s}s …")
        parser = FrameParser()
        make_tx = make_downlink
    print("  同时看 F407 USB 调试口是否出现 [JETSON CMD] seq=… v=0 …\n")
    deadline = time.monotonic() + duration_s
    seq = 0
    tx_count = 0
    rx_status = 0
    rx_ext = 0
    last_print = 0.0

    while time.monotonic() < deadline:
        ser.write(make_tx(seq))
        tx_count += 1
        seq = (seq + 1) & 0xFF

        n = ser.in_waiting
        if n > 0:
            for p in parser.feed(ser.read(n)):
                if blob_v2:
                    if p["type"] == MSG_AGV_MOTION:
                        rx_status += 1
                    elif p["type"] == MSG_MCU_STATUS:
                        rx_ext += 1
                    now = time.monotonic()
                    if now - last_print >= 0.5:
                        if p["type"] == MSG_MCU_STATUS:
                            hb = "丢失" if p["link_state"] & 0x01 else "正常"
                            print(
                                f"  TX seq={seq - 1} | RX 0x03 seq={p['seq']} "
                                f"仲裁={p['safety_name']} Jetson心跳={hb} "
                                f"sonarF={p['sonar_front']}"
                            )
                        elif p["type"] == MSG_AGV_MOTION:
                            print(
                                f"  TX seq={seq - 1} | RX 0x02 v={p['v_mm_s']}mm/s "
                                f"bat={p['bat_v'] * 0.1:.1f}V"
                            )
                        last_print = now
                else:
                    if p["type"] == FRAME_TYPE_UP_STATUS:
                        rx_status += 1
                    else:
                        rx_ext += 1
                    now = time.monotonic()
                    if now - last_print >= 0.5:
                        if p["type"] == FRAME_TYPE_UP_STATUS:
                            hb = "丢失" if p["link_state"] & 0x01 else "正常"
                            print(
                                f"  TX seq={seq - 1} | RX 0x02 seq={p['seq']} "
                                f"仲裁={p['safety_name']} Jetson心跳={hb} "
                                f"sonarF={p['sonar_front']}"
                            )
                        else:
                            print(f"  TX seq={seq - 1} | RX 0x03 seq={p['seq']}")
                        last_print = now

        time.sleep(0.02)

    if blob_v2:
        print(f"\n统计: 下发 {tx_count} 帧, 收到 0x02={rx_status}, 0x03={rx_ext}")
        if rx_status == 0 and rx_ext == 0:
            print("❌ 步骤 B 未通过：发了 BLOB 心跳但无上行。")
        elif tx_count > 0 and (rx_status > 0 or rx_ext > 0):
            print("✅ 步骤 B 通过：BLOB v2 双向通信正常。")
    else:
        print(f"\n统计: 下发 {tx_count} 帧, 收到 0x02={rx_status}, 0x03={rx_ext}")
        if rx_status == 0:
            print("❌ 步骤 B 未通过：发了心跳但无 0x02 上行 → 查 TX/RX 是否交叉、共地、波特率。")
        elif tx_count > 0 and rx_status > 0:
            print("✅ 步骤 B 通过：双向通信正常。")
            print("  下一步：看 F407 调试口 ARB 是否 DEGRADED→RECOVERING→NORMAL（约 1s）。")


def main() -> None:
    ap = argparse.ArgumentParser(description="Jetson↔STM32B RS232 链路测试")
    ap.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0",
        help="STM32B 串口（Prolific USB-TTL）",
    )
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--time", type=float, default=10.0, help="测试时长(秒)")
    ap.add_argument(
        "--listen-only",
        action="store_true",
        help="步骤 A：只收不发",
    )
    ap.add_argument(
        "--blob-v2",
        action="store_true",
        help="使用 BLOB v2 协议（0xAB 头，MCU JETSON_USE_BLOB_V2=1）",
    )
    args = ap.parse_args()

    print("=" * 60)
    print("Jetson ↔ STM32B RS232 链路测试")
    print(f"  协议: {'BLOB v2 (0xAB)' if args.blob_v2 else 'V3 (0xAA)'}")
    print(f"  端口: {args.port}")
    print(f"  波特率: {args.baud} 8N1")
    print("  接线: Jetson TX→STM32 PA3(RX), Jetson RX←STM32 PA2(TX), GND 共地")
    print("=" * 60)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except (serial.SerialException, OSError) as e:
        print(f"\n❌ 打不开串口: {e}")
        print("  · 确认 USB 已插、设备存在: ls -l /dev/serial/by-id/")
        print("  · 是否被占用: fuser -v /dev/ttyUSB6")
        print("  · 先停 jetson_bridge: pkill -f jetson_bridge")
        sys.exit(1)

    try:
        if args.listen_only:
            run_listen_only(ser, args.time, args.blob_v2)
        else:
            run_heartbeat(ser, args.time, args.blob_v2)
    except KeyboardInterrupt:
        print("\n已停止。")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
