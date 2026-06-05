#!/usr/bin/env python3
"""Scan ttyUSB/ttyACM ports: show USB identity and probe for STM32 0xFF frames."""

import argparse
import glob
import os
import struct
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    print("请先安装: sudo apt install python3-serial", file=sys.stderr)
    sys.exit(1)

from ds_serial_monitor.protocol import FRAME_HEADER, FRAME_LEN, parse_frame


def udev_props(dev: str) -> dict[str, str]:
    props: dict[str, str] = {}
    try:
        out = subprocess.check_output(
            ["udevadm", "info", "-q", "property", "-n", dev],
            text=True,
            stderr=subprocess.DEVNULL,
        )
        for line in out.splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                props[k] = v
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    return props


def list_ports() -> list[str]:
    ports = sorted(
        glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyACM*")
        + glob.glob("/dev/ttyTHS*")
    )
    return ports


def probe_port(port: str, baud: int, seconds: float) -> tuple[int, dict | None]:
    """Return (bytes_read, last_valid_frame_or_none)."""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.05,
        )
    except (serial.SerialException, OSError) as e:
        return -1, {"error": str(e)}

    buf = bytearray()
    last_ok = None
    total = 0
    deadline = time.monotonic() + seconds

    try:
        while time.monotonic() < deadline:
            n = ser.in_waiting
            if n <= 0:
                time.sleep(0.02)
                continue
            chunk = ser.read(n)
            total += len(chunk)
            buf.extend(chunk)
            while len(buf) >= FRAME_LEN:
                if buf[0] != FRAME_HEADER:
                    buf.pop(0)
                    continue
                frame = bytes(buf[:FRAME_LEN])
                del buf[:FRAME_LEN]
                parsed = parse_frame(frame)
                if parsed:
                    last_ok = parsed
    finally:
        ser.close()

    return total, last_ok


def main() -> None:
    ap = argparse.ArgumentParser(description="Find STM32 USART3 serial port")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-t", "--time", type=float, default=2.0, help="Seconds per port")
    ap.add_argument(
        "--only",
        default="",
        help="Only probe ports matching substring (e.g. USB5 or Prolific)",
    )
    args = ap.parse_args()

    ports = list_ports()
    if args.only:
        ports = [p for p in ports if args.only in p or args.only.lower() in udev_props(p).get("ID_VENDOR", "").lower()]
    if not ports:
        print("未找到 /dev/ttyUSB* 或 /dev/ttyACM*")
        print("若 lsusb 有 PL2303/CH340 但无 tty：尝试 sudo modprobe pl2303")
        sys.exit(1)

    print("=" * 70)
    print("串口设备列表（拔插 STM32 前后对比 ID_PATH / 厂商）")
    print("=" * 70)

    for port in ports:
        props = udev_props(port)
        vendor = props.get("ID_VENDOR", "?")
        model = props.get("ID_MODEL", "?")
        path = props.get("ID_PATH", "?")
        serial_id = props.get("ID_SERIAL_SHORT", props.get("ID_SERIAL", "?"))
        print(f"\n{port}")
        print(f"  厂商/型号: {vendor} / {model}")
        print(f"  序列号:    {serial_id}")
        print(f"  物理路径:  {path}")

    print("\n" + "=" * 70)
    print(f"探测 STM32 帧 (0xFF, {FRAME_LEN}B, XOR) — 每口 {args.time}s @ {args.baud}")
    print("请确认 STM32 正在通过 USART3 发送数据")
    print("=" * 70)

    hits: list[str] = []
    for port in ports:
        nbytes, parsed = probe_port(port, args.baud, args.time)
        if nbytes < 0:
            print(f"\n{port}: 无法打开 — {parsed.get('error')}")
            continue
        if parsed:
            d = parsed["distances_raw"]
            print(
                f"\n{port}: *** 匹配 STM32 协议 ***  "
                f"IF=[{d[0]},{d[1]},{d[2]},{d[3]}] mm beep={parsed['beep_duty']}%"
            )
            hits.append(port)
        else:
            print(f"\n{port}: 收到 {nbytes} 字节，无有效 0xFF 帧")

    print("\n" + "=" * 70)
    if hits:
        print("建议使用:")
        for p in hits:
            print(f"  ros2 run ds_serial_monitor serial_monitor -- -p {p}")
    else:
        print("未找到 STM32 数据帧。可能原因:")
        print("  1. ttyUSB0-4 是 Quectel 4G 模组，不是 STM32")
        print("  2. PL2303: sudo bash ~/catkin_ws/src/ds_serial_monitor/scripts/build_and_install_pl2303.sh")
        print("     成功后用 /dev/ttyUSB5 (Prolific)")
        print("  3. 或 STM32 USART 接 Jetson 引脚 UART → /dev/ttyTHS1 等")
        print("  4. 只接 CAN → 用 ds_can_monitor（推荐，已验证）")
        print("  4. 波特率不是 115200 或 STM32 未发送")
    print("=" * 70)


if __name__ == "__main__":
    main()
