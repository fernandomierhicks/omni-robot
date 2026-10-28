"""The servo bus's byte pipe (design.md 11): the bus board's USB serial port on the robot, the sim's bus over localhost
TCP (simulation/v11 sil/arm_bus.ServoPort), both with write(bytes) / read(n) that never blocks."""

from __future__ import annotations

import socket


class TcpPort:
    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.s.setblocking(False)

    def write(self, data: bytes):
        self.s.setblocking(True)
        self.s.sendall(data)
        self.s.setblocking(False)

    def read(self, n: int = 256) -> bytes:
        try:
            return self.s.recv(n)
        except (BlockingIOError, InterruptedError):
            return b""

    def close(self):
        self.s.close()


class SerialPort:
    def __init__(self, dev: str, baud: int):
        import serial                                  # pyserial (python3-serial)
        self.s = serial.Serial(dev, baud, timeout=0, write_timeout=0.05)

    def write(self, data: bytes):
        self.s.write(data)

    def read(self, n: int = 256) -> bytes:
        return self.s.read(n)

    def close(self):
        self.s.close()


def open_port(spec: str, baud: int = 1_000_000):
    """"tcp:host:port" (the sim) or a device ("/dev/ttyACM0", "COM5")."""
    if spec.startswith("tcp:"):
        _, host, port = spec.split(":")
        return TcpPort(host, int(port))
    return SerialPort(spec, baud)
