"""串口打开、重连与读写。"""

from __future__ import annotations

import threading
import time

try:
    import serial
except ImportError as exc:
    raise RuntimeError("Install: sudo apt install python3-serial") from exc


class SerialLink:
    def __init__(
        self,
        port: str,
        baud: int = 115200,
        *,
        settle_s: float = 0.15,
        skip_dtr_reset: bool = True,
        no_flush_on_open: bool = True,
        logger=None,
    ) -> None:
        self._port = port
        self._baud = baud
        self._settle_s = settle_s
        self._skip_dtr_reset = skip_dtr_reset
        self._no_flush_on_open = no_flush_on_open
        self._logger = logger
        self._ser: serial.Serial | None = None
        self._write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def port(self) -> str:
        return self._port

    def open(self) -> bool:
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
                dsrdtr=False,
                rtscts=False,
            )
            if not self._no_flush_on_open:
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
            if not self._skip_dtr_reset:
                try:
                    self._ser.dtr = False
                    time.sleep(0.05)
                    self._ser.dtr = True
                except Exception:
                    pass
            time.sleep(self._settle_s)
            return True
        except OSError as exc:
            self._ser = None
            if self._logger:
                self._logger.warn(f"串口打开失败 ({self._port}): {exc}")
            return False

    def close(self) -> None:
        if self._ser is None:
            return
        try:
            self._ser.close()
        except Exception:
            pass
        self._ser = None

    def write(self, data: bytes) -> None:
        if self._ser is None:
            raise OSError("serial closed")
        with self._write_lock:
            self._ser.write(data)

    def read_available(self) -> bytes:
        if self._ser is None:
            return b""
        try:
            n = self._ser.in_waiting
            if n <= 0:
                return b""
            return self._ser.read(n)
        except (OSError, serial.SerialException) as exc:
            if self._logger:
                self._logger.warn(f"串口读失败(保持连接): {exc}", throttle_duration_sec=5.0)
            return b""

    def flush_rx(self, discard_s: float = 0.2) -> int:
        if self._ser is None:
            return 0
        discarded = 0
        old_timeout = self._ser.timeout
        self._ser.timeout = 0.05
        deadline = time.monotonic() + discard_s
        try:
            while time.monotonic() < deadline:
                chunk = self._ser.read(512)
                if chunk:
                    discarded += len(chunk)
                else:
                    time.sleep(0.01)
            self._ser.reset_input_buffer()
        except (OSError, serial.SerialException) as exc:
            if self._logger:
                self._logger.warn(f"flush_rx 失败(保持连接): {exc}", throttle_duration_sec=5.0)
        finally:
            if self._ser is not None:
                self._ser.timeout = old_timeout
        return discarded
