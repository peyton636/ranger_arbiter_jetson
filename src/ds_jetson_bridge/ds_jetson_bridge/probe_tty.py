#!/usr/bin/env python3
"""Probe ttyUSB: detect V3 uplink 0x02 and test V3 downlink 0x01."""

import argparse
import glob
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    print("sudo apt install python3-serial", file=sys.stderr)
    sys.exit(1)

from ds_jetson_bridge.jetson_protocol import (
    FRAME_LEN,
    FRAME_TYPE_UP_STATUS,
    encode_stop_downlink,
    parse_uplink_status,
    FrameParser,
)

MODEM_VENDORS = frozenset({"Quectel", "QUALCOMM", "Sierra", "Huawei"})


def udev_vendor(port: str) -> str:
    try:
        out = subprocess.check_output(
            ["udevadm", "info", "-q", "property", "-n", port],
            text=True,
            stderr=subprocess.DEVNULL,
        )
        for line in out.splitlines():
            if line.startswith("ID_VENDOR="):
                return line.split("=", 1)[1]
    except Exception:
        pass
    return "?"


def probe_uplink(port: str, baud: int, seconds: float) -> int:
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except (serial.SerialException, OSError) as e:
        print(f"  {port}: open failed: {e}")
        return 0

    parser = FrameParser()
    deadline = time.monotonic() + seconds
    good = 0
    while time.monotonic() < deadline:
        n = ser.in_waiting
        if n > 0:
            for frame in parser.feed(ser.read(n)):
                if frame[1] == FRAME_TYPE_UP_STATUS and parse_uplink_status(frame):
                    good += 1
        time.sleep(0.02)
    ser.close()
    return good


def main() -> None:
    ap = argparse.ArgumentParser(description="Find STM32B tty (V3 24B)")
    ap.add_argument("-t", "--time", type=float, default=3.0)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    ports = sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        print("No /dev/ttyUSB* found")
        sys.exit(1)

    print("=" * 60)
    print("ttyUSB probe V3 (uplink 0x02 status, downlink 0x01 test)")
    print("=" * 60)

    uplink_ports: list[str] = []
    for port in ports:
        vendor = udev_vendor(port)
        print(f"\n{port}  vendor={vendor}")
        n = probe_uplink(port, args.baud, args.time)
        print(f"  uplink 0x02 @ {args.baud}: {n} valid frames in {args.time}s")
        if n > 0:
            uplink_ports.append(port)

    prolific = [p for p in ports if "Prolific" in udev_vendor(p)]
    non_modem = [p for p in ports if udev_vendor(p) not in MODEM_VENDORS]

    if uplink_ports:
        cmd = sensor = uplink_ports[0]
    elif prolific:
        cmd = sensor = prolific[0]
    elif non_modem:
        cmd = sensor = non_modem[0]
    else:
        cmd = sensor = "/dev/ttyUSB5"

    print("\n" + "=" * 60)
    print("Suggested:")
    print(f"  cmd_port:={cmd}")
    print(f"  sensor (legacy 12B debug):={sensor}")
    if not uplink_ports:
        print("  (no 0x02 yet — STM32B V3 uplink not running or wrong baud)")
    print("  Do NOT use Quectel ttyUSB0-4 for STM32.")
    print("=" * 60)

    seq = 0
    for port in ports:
        try:
            ser = serial.Serial(port, args.baud, timeout=0.1)
            ser.write(encode_stop_downlink(seq))
            note = " (modem)" if udev_vendor(port) in MODEM_VENDORS else ""
            print(f"  downlink test OK on {port}{note}")
            ser.close()
        except Exception as e:
            print(f"  downlink test FAIL on {port}: {e}")


if __name__ == "__main__":
    main()
