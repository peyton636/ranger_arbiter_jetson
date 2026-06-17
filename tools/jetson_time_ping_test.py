#!/usr/bin/env python3
"""
Jetson ↔ STM32 时间同步联调（START / PING / QUERY / STOP + RTT / offset）

不依赖 ROS，纯 Python 串口测试。

接线：Jetson RS232 接 STM32 USART2 (PA2/PA3)，不是 Windows 调试口 USART1。

推荐测试顺序：
  1. fuser -v /dev/ttyUSB7          # 确认无 gateway 占口
  2. --probe                       # 应看到大量 0xAA(V3)，不是 ASCII 日志
  3. --query-only                  # 验证固件 + 接线
  4. --count 20 --query            # START/PING 全流程

推荐端口（STM32B Prolific，插拔后编号可能变，优先 by-id）：
  /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
  → 当前多为 /dev/ttyUSB7
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("请先安装: sudo apt install python3-serial", file=sys.stderr)
    sys.exit(1)

sys.path.insert(0, "src/drivers/rs232_gateway")

from rs232_gateway.service_frame import CAN_ID_TIME_SYNC_RSP, Rs232StreamParser
from rs232_gateway.time_sync import CMD_PING, CMD_START, JetsonTimeSync

DEFAULT_PORT = (
    "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"
)


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
    )
    ser.reset_input_buffer()
    time.sleep(0.15)
    return ser


def scan_108_payloads(buf: bytes) -> list[bytes]:
    """在原始字节流中扫描所有 0xA5 0x01 0x08 服务帧（抗 V3 混传干扰）。"""
    out: list[bytes] = []
    i = 0
    while i + 11 <= len(buf):
        if buf[i] == 0xA5 and buf[i + 1] == 0x01 and buf[i + 2] == 0x08:
            out.append(bytes(buf[i + 3 : i + 11]))
            i += 11
        else:
            i += 1
    return out


def tx_and_wait_108(
    ser: serial.Serial,
    frame: bytes,
    timeout_s: float,
    *,
    expect_cmd: int | None = None,
    expect_echo: int | None = None,
    is_query: bool = False,
) -> tuple[bytes | None, int, list[bytes], float | None]:
    """发 0x107，收 0x108。返回 (payload, rx_bytes, all_108, t_rx_mono)。"""
    ser.reset_input_buffer()
    ser.write(frame)
    deadline = time.monotonic() + timeout_s
    raw = bytearray()
    t_rx: float | None = None
    while time.monotonic() < deadline:
        chunk = ser.read(4096)
        if chunk:
            raw.extend(chunk)
            payloads = scan_108_payloads(raw)
            if payloads:
                pick = _pick_108_payload(
                    payloads, expect_cmd=expect_cmd, expect_echo=expect_echo, is_query=is_query
                )
                if pick is not None:
                    t_rx = time.monotonic()
                    return pick, len(raw), payloads, t_rx
        else:
            time.sleep(0.002)
    payloads = scan_108_payloads(raw)
    pick = _pick_108_payload(
        payloads, expect_cmd=expect_cmd, expect_echo=expect_echo, is_query=is_query
    )
    if pick is not None:
        t_rx = time.monotonic()
    return pick, len(raw), payloads, t_rx


def _pick_108_payload(
    payloads: list[bytes],
    *,
    expect_cmd: int | None,
    expect_echo: int | None,
    is_query: bool,
) -> bytes | None:
    if not payloads:
        return None
    if is_query:
        return payloads[-1]
    if expect_cmd is not None:
        for p in payloads:
            if p[0] == expect_cmd and (
                expect_echo is None or p[1] == (expect_echo & 0xFF)
            ):
                return p
        for p in payloads:
            if p[0] == expect_cmd:
                return p
    return payloads[-1]


def fmt_108_list(payloads: list[bytes], limit: int = 5) -> str:
    if not payloads:
        return "[]"
    parts = [p.hex() for p in payloads[:limit]]
    suffix = f" ...+{len(payloads) - limit}" if len(payloads) > limit else ""
    return "[" + ", ".join(parts) + suffix + "]"


def run_probe(ser: serial.Serial, duration_s: float) -> int:
    """只收不发：统计 V3 / 服务帧 / ASCII 比例。"""
    parser = Rs232StreamParser()
    deadline = time.monotonic() + duration_s
    total_bytes = 0
    ascii_bytes = 0
    v3_count = 0
    svc_count = 0
    svc_108 = 0

    print(f"侦听 {duration_s:.1f}s（只收不发）...")
    while time.monotonic() < deadline:
        chunk = ser.read(512)
        if not chunk:
            time.sleep(0.01)
            continue
        total_bytes += len(chunk)
        for b in chunk:
            if 0x20 <= b <= 0x7e or b in (0x09, 0x0a, 0x0d):
                ascii_bytes += 1
        for item in parser.feed(chunk):
            if item[0] == "v3":
                v3_count += 1
            else:
                svc_count += 1
                if item[1] == CAN_ID_TIME_SYNC_RSP:
                    svc_108 += 1

    ascii_pct = (100.0 * ascii_bytes / total_bytes) if total_bytes else 0.0
    print(f"  总字节: {total_bytes}")
    print(f"  V3 0xAA 帧: {v3_count}")
    print(f"  服务帧 0xA5: {svc_count} (其中 0x108: {svc_108})")
    print(f"  可打印 ASCII 占比: {ascii_pct:.1f}%")

    if v3_count > 0 and ascii_pct < 30:
        print("  结论: 正常 — 这是 USART2 二进制链路（V3 上行）")
        return 0
    if ascii_pct > 70:
        print("  结论: 疑似接到 USART1 调试口或错误端口（大量 ASCII 日志）")
        return 2
    if v3_count == 0:
        print("  结论: 无 V3 上行 — 查接线 PA2/PA3、波特率、MCU 是否运行")
        return 2
    print("  结论: 有数据但需人工判断")
    return 1


def run_query_only(ser: serial.Serial, ts: JetsonTimeSync, timeout_s: float) -> int:
    frame = ts.build_query()
    payload, rx_total, all108, t_rx = tx_and_wait_108(
        ser, frame, timeout_s, is_query=True
    )
    if payload is None:
        print(f"QUERY: 超时无 0x108（收到 {rx_total} 字节，扫描到 0x108={len(all108)}）")
        if rx_total == 0:
            print("  → 串口完全无回包：查 STM32 上电、PA2/PA3 接线、TX/RX 是否交叉、共地")
        else:
            print(f"  → 有 {rx_total} 字节但无 0xA5/01/08 帧")
        return 2
    upd = ts.on_response(payload)
    if upd is None:
        print(f"QUERY: 解析失败 raw={payload.hex()}")
        return 2
    print(
        f"QUERY ok: mcu_tick={upd.system_tick_ms} utc_unix={upd.utc_unix_sec} "
        f"raw={payload.hex()}"
    )
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Jetson TimeSync 联调")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--count", type=int, default=20, help="PING 次数（不含 burst）")
    ap.add_argument("--timeout", type=float, default=1.0, help="等 0x108 超时(秒)")
    ap.add_argument("--session", type=int, default=1)
    ap.add_argument("--no-start", action="store_true", help="跳过 START，只测 PING")
    ap.add_argument("--ping-only", action="store_true", help="跳过 START，等同 --no-start")
    ap.add_argument("--query", action="store_true", help="结束时发 QUERY")
    ap.add_argument(
        "--probe",
        action="store_true",
        help="只收不发，统计 V3/ASCII（验证是否 USART2 二进制链路）",
    )
    ap.add_argument("--probe-time", type=float, default=5.0, help="--probe 侦听秒数")
    ap.add_argument(
        "--query-only",
        action="store_true",
        help="只发一次 QUERY，验证固件与接线",
    )
    args = ap.parse_args()

    if not args.port or not str(args.port).strip():
        print("错误: --port 为空。请先设置 PORT 变量或直接写路径。", file=sys.stderr)
        print("  例: PORT=/dev/serial/by-id/usb-Prolific_...-port0", file=sys.stderr)
        print("  或: --port /dev/ttyUSB7", file=sys.stderr)
        return 1

    print(f"打开串口 {args.port} @ {args.baud}")
    try:
        ser = open_serial(args.port, args.baud)
    except OSError as exc:
        print(f"串口打开失败: {exc}", file=sys.stderr)
        print("提示: ls -la /dev/serial/by-id/  查 Prolific 对应端口", file=sys.stderr)
        print("      fuser -v /dev/ttyUSB7  查是否被 gateway 占用", file=sys.stderr)
        return 1

    try:
        if args.probe:
            return run_probe(ser, args.probe_time)

        ts = JetsonTimeSync(session_id=args.session, ping_burst_count=0)
        timeout_s = args.timeout

        if args.query_only:
            return run_query_only(ser, ts, timeout_s)

        ok = 0
        fail = 0
        skip_start = args.no_start or args.ping_only

        if not skip_start:
            frame = ts.begin_session()
            payload, rx_total, all108, t_rx = tx_and_wait_108(
                ser,
                frame,
                timeout_s,
                expect_cmd=CMD_START,
                expect_echo=args.session,
            )
            if payload is None:
                print(
                    f"START: 超时无 0x108（收到 {rx_total} 字节，"
                    f"0x108帧={len(all108)} {fmt_108_list(all108)}）"
                )
                fail += 1
            else:
                t4_ms = t_rx * 1000.0 if t_rx is not None else None
                upd = ts.on_response(payload, t4_ms=t4_ms)
                rtt_s = f"rtt={upd.rtt_ms:.2f}ms" if upd else ""
                print(
                    f"START OK: cmd_echo=0x{payload[0]:02X} seq={payload[1]} "
                    f"{rtt_s} offset={ts.offset_ms:.2f}ms "
                    f"(offset=mcu_tick-jetson_mono，大负数正常)"
                )
                ok += 1

        for i in range(args.count):
            frame = ts.build_ping()
            ping_seq = ts.ping_seq
            payload, rx_total, all108, t_rx = tx_and_wait_108(
                ser,
                frame,
                timeout_s,
                expect_cmd=CMD_PING,
                expect_echo=ping_seq,
            )
            if payload is None:
                print(
                    f"PING #{i+1} seq={ping_seq}: 超时（收到 {rx_total} 字节，"
                    f"0x108帧={len(all108)} {fmt_108_list(all108)}）"
                )
                if rx_total < 1000 and i == 0 and not skip_start:
                    print(
                        "  → START 后 V3 上行骤减：MCU 可能在 START 后阻塞 USART2，"
                        "查固件 Jetson 任务 / [JETSON TIME] cmd=0x03"
                    )
                fail += 1
                continue
            t4_ms = t_rx * 1000.0 if t_rx is not None else None
            upd = ts.on_response(payload, t4_ms=t4_ms)
            if upd is None:
                print(f"PING #{i+1}: 解析失败 {payload.hex()}")
                fail += 1
                continue
            ok += 1
            print(
                f"PING #{i+1}: rtt={upd.rtt_ms:.2f}ms offset={upd.offset_ms:.2f}ms "
                f"mcu_tick={upd.mcu_tick_rx} proc={upd.proc_ms:.2f}ms "
                f"echo=0x{upd.cmd_echo:02X} seq={upd.seq_echo}"
            )
            time.sleep(0.1)

        if args.query:
            frame = ts.build_query()
            payload, rx_total, all108, t_rx = tx_and_wait_108(
                ser, frame, timeout_s, is_query=True
            )
            if payload is None:
                print(
                    f"QUERY: 超时（收到 {rx_total} 字节，0x108帧={len(all108)}）"
                )
                fail += 1
            else:
                upd = ts.on_response(payload)
                print(
                    f"QUERY OK: tick={upd.system_tick_ms} utc={upd.utc_unix_sec}"
                    if upd
                    else f"QUERY raw={payload.hex()}"
                )
                ok += 1

        if ts.session_active:
            ser.reset_input_buffer()
            ser.write(ts.stop_session())
        print(
            f"\n汇总: ok={ok} fail={fail} final_offset={ts.offset_ms:.2f}ms "
            f"rtt_ema={ts.rtt_ema_ms:.2f}ms"
        )
        return 0 if fail == 0 else 2
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
