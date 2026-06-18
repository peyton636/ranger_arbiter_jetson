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
        self._rx: socket.socket | None = None
        self._tx: socket.socket | None = None

    @property
    def connected(self) -> bool:
        return self._rx is not None

    def open(self) -> bool:
        self.close()
        try:
            rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            rx.bind((self.bind_ip, self.local_port))
            rx.setblocking(False)
            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._rx = rx
            self._tx = tx
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
        for sock in (self._rx, self._tx):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._rx = None
        self._tx = None

    def send(self, data: bytes) -> None:
        if self._tx is None:
            raise OSError("UDP not open")
        self._tx.sendto(data, (self.mcu_ip, self.mcu_port))

    def recv_available(self) -> list[bytes]:
        if self._rx is None:
            return []
        out: list[bytes] = []
        while True:
            try:
                data, _addr = self._rx.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break
            if data:
                out.append(data)
        return out
