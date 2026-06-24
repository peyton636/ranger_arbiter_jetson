"""UDP transport for MCU BLOB v2 (one datagram = one wire frame)."""

from __future__ import annotations

import socket
from typing import Callable


class UdpLink:
    def __init__(
        self,
        bind_ip: str,
        local_port: int,
        mcu_ip: str,
        mcu_port: int,
        *,
        logger: Callable[..., None] | None = None,
    ) -> None:
        self.bind_ip = bind_ip
        self.local_port = local_port
        self.mcu_ip = mcu_ip
        self.mcu_port = mcu_port
        self._log = logger or (lambda *args, **kwargs: None)
        self._sock: socket.socket | None = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def open(self) -> bool:
        self.close()
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # 单 socket 绑定 bind_ip:local_port，收发共用，确保下行源 IP 与 bind_ip 一致
            sock.bind((self.bind_ip, self.local_port))
            sock.setblocking(False)
            self._sock = sock
            self._log(
                f"UDP bind {self.bind_ip}:{self.local_port}, "
                f"peer {self.mcu_ip}:{self.mcu_port}"
            )
            return True
        except OSError as exc:
            self._log(f"UDP open failed: {exc}")
            self.close()
            return False

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None

    def send(self, data: bytes) -> None:
        if self._sock is None:
            raise OSError("UDP not open")
        self._sock.sendto(data, (self.mcu_ip, self.mcu_port))

    def recv_available(self) -> list[bytes]:
        if self._sock is None:
            return []
        out: list[bytes] = []
        while True:
            try:
                data, _addr = self._sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break
            if data:
                out.append(data)
        return out
