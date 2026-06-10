#!/usr/bin/env python3
"""Read CAN from SocketCAN, parse 0x110/0x111, refresh terminal display."""

import argparse
import select
import socket
import struct
import sys
import time

from ds_can_monitor.protocol import (
    CAN_ID_MAIN,
    CAN_ID_SENSOR,
    CAN_ID_TEST,
    parse_main_frame,
    parse_sensor_frame,
)

CAN_FRAME_FMT = "=IB3x8s"  # can_id, dlc, pad(3), data[8] — 3x is padding, not a field
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_SFF_MASK = 0x7FF


class CanSocketReader:
    def __init__(self, interface: str):
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.bind((interface,))
        self._sock.setblocking(False)

    def recv(self, timeout_s: float = 0.05) -> tuple[int, bytes] | None:
        ready, _, _ = select.select([self._sock], [], [], timeout_s)
        if not ready:
            return None
        frame = self._sock.recv(CAN_FRAME_SIZE)
        if len(frame) < CAN_FRAME_SIZE:
            return None
        can_id, length, data = struct.unpack(CAN_FRAME_FMT, frame)
        can_id &= CAN_SFF_MASK
        return can_id, data[:length]

    def close(self) -> None:
        self._sock.close()


class SensorCanState:
    def __init__(self) -> None:
        self.main: dict | None = None
        self.sensor: dict | None = None
        self.test_counter: int | None = None
        self.frame_count = 0

    def update(self, can_id: int, data: bytes) -> bool:
        """Return True when a full display refresh is worthwhile."""
        if can_id == CAN_ID_MAIN and len(data) >= 8:
            self.main = parse_main_frame(data)
            return False
        if can_id == CAN_ID_SENSOR and len(data) >= 8:
            self.sensor = parse_sensor_frame(data)
            self.frame_count += 1
            return True
        if can_id == CAN_ID_TEST and len(data) >= 1:
            self.test_counter = data[0]
            return False
        return False


def format_display(state: SensorCanState, interface: str) -> str:
    lines: list[str] = []
    lines.append("=" * 60)
    lines.append(f"  DS CAN Monitor  |  {interface}  |  Ctrl+C exit")
    lines.append("=" * 60)

    if state.main:
        m = state.main
        lines.append(
            f"[Status] Version:0x{m['version']:02X}  Sensors:{m['sensor_count']}  "
            f"Beep:{m['beep_volume']}%  Status:0x{m['status']:02X}"
        )
        flags = []
        if m["data_valid"]:
            flags.append("DATA_VALID")
        if m["can_ok"]:
            flags.append("CAN_OK")
        lines.append(f"         Flags: {' | '.join(flags) if flags else '(none)'}")
    else:
        lines.append("[Status] waiting for 0x110 ...")

    if state.sensor:
        s = state.sensor
        for i in range(4):
            dist = s["distances"][i]
            ok = s["valid"][i]
            mark = "OK" if ok else "INVALID"
            if ok:
                lines.append(f"[DS] IF{i + 1}: {dist} mm  ({mark})")
            else:
                lines.append(f"[DS] IF{i + 1}: {dist} mm  ({mark}, filtered)")
        lines.append("---")
        if s["nearest_mm"] is not None:
            beep = state.main["beep_volume"] if state.main else "?"
            lines.append(
                f"[DS BEEP] Nearest: {s['nearest_mm']} mm, PWM: {beep}%"
            )
            if s["nearest_mm"] < 500:
                lines.append(
                    f"!! WARNING: obstacle < 50cm ({s['nearest_mm']} mm) !!"
                )
        else:
            lines.append("[DS BEEP] Nearest: (no valid sensor)")
    else:
        lines.append("[Sensor] waiting for 0x111 ...")

    if state.test_counter is not None:
        lines.append(f"[Test] 0x7FF counter: 0x{state.test_counter:02X}")

    lines.append(f"[Stats] sensor frames: {state.frame_count}")
    lines.append("")
    return "\n".join(lines)


def clear_and_print(text: str) -> None:
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.write(text)
    sys.stdout.flush()


def run_monitor(interface: str, refresh_hz: float) -> None:
    reader = CanSocketReader(interface)
    state = SensorCanState()
    min_interval = 1.0 / refresh_hz if refresh_hz > 0 else 0.0
    last_draw = 0.0

    print(f"Listening on {interface} (IDs 0x110, 0x111, 0x7FF)...")
    time.sleep(0.5)

    try:
        while True:
            msg = reader.recv(0.05)
            now = time.monotonic()
            if msg is None:
                continue
            can_id, data = msg
            if state.update(can_id, data):
                if min_interval <= 0 or (now - last_draw) >= min_interval:
                    clear_and_print(format_display(state, interface))
                    last_draw = now
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        reader.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Parse STM32 CAN sensor data on SocketCAN"
    )
    parser.add_argument(
        "-i",
        "--interface",
        default="can2",
        help="SocketCAN interface (default: can2)",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=10.0,
        help="Max display refresh rate (default: 10)",
    )
    args = parser.parse_args()
    run_monitor(args.interface, args.hz)


if __name__ == "__main__":
    main()
