#!/usr/bin/env python3
"""Receive STM32 USART3 frames over USB serial and refresh terminal display."""

import argparse
import sys
import time

try:
    import serial
except ImportError as exc:
    print(
        "缺少 pyserial，请先安装:\n"
        "  sudo apt install python3-serial\n",
        file=sys.stderr,
    )
    raise SystemExit(1) from exc

from ds_serial_monitor.protocol import (
    FRAME_LEN,
    FRAME_HEADER,
    FrameParser,
)


def format_display(parsed: dict, port: str, frame_no: int) -> str:
    lines: list[str] = []
    lines.append("=" * 60)
    lines.append(f"  DS Serial Monitor  |  {port} 115200 8N1  |  Ctrl+C exit")
    lines.append("=" * 60)
    lines.append(
        f"[Frame] #{frame_no}  sensors={parsed['sensor_count']}  "
        f"beep={parsed['beep_duty']}%  checksum=OK"
    )

    for i in range(4):
        raw = parsed["distances_raw"][i]
        if parsed["valid"][i]:
            lines.append(f"[DS] IF{i + 1}: {parsed['distances'][i]} mm  (OK)")
        else:
            lines.append(f"[DS] IF{i + 1}: {raw} mm  (INVALID)")

    lines.append("---")
    if parsed["nearest_mm"] is not None:
        lines.append(
            f"[DS BEEP] Nearest: {parsed['nearest_mm']} mm, PWM: {parsed['beep_duty']}%"
        )
        if parsed["nearest_mm"] < 500:
            lines.append(
                f"!! WARNING: obstacle < 50cm ({parsed['nearest_mm']} mm) !!"
            )
    else:
        lines.append("[DS BEEP] Nearest: (no valid sensor)")

    lines.append("")
    return "\n".join(lines)


def clear_and_print(text: str) -> None:
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.write(text)
    sys.stdout.flush()


def run_monitor(port: str, baud: int, refresh_hz: float) -> None:
    parser = FrameParser()
    frame_no = 0
    min_interval = 1.0 / refresh_hz if refresh_hz > 0 else 0.0
    last_draw = 0.0

    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
    )

    print(f"Opened {port} @ {baud} 8N1, frame {FRAME_LEN}B header=0x{FRAME_HEADER:02X}")
    time.sleep(0.3)

    try:
        while True:
            waiting = ser.in_waiting
            if waiting <= 0:
                time.sleep(0.01)
                continue

            chunk = ser.read(waiting)
            for parsed in parser.feed(chunk):
                frame_no += 1
                now = time.monotonic()
                if min_interval <= 0 or (now - last_draw) >= min_interval:
                    clear_and_print(format_display(parsed, port, frame_no))
                    last_draw = now
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


def main() -> None:
    argp = argparse.ArgumentParser(
        description="STM32 USART3 12-byte sensor frame receiver"
    )
    argp.add_argument(
        "-p",
        "--port",
        default="/dev/ttyUSB0",
        help="Serial port: /dev/ttyUSBx (USB-RS232) or /dev/ttyTHSx (Jetson UART)",
    )
    argp.add_argument(
        "-b",
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    argp.add_argument(
        "--hz",
        type=float,
        default=10.0,
        help="Max terminal refresh rate (default: 10)",
    )
    args = argp.parse_args()
    run_monitor(args.port, args.baud, args.hz)


if __name__ == "__main__":
    main()
