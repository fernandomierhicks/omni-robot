"""The SIL firmware's USB port as a localhost TCP server (design.md 30): omni_mcu's mcu_link connects to it exactly as
it opens /dev/ttyACM0 on the robot. Bytes go in and out once per firmware tick, so the link sees the Teensy's 1 ms
rhythm; with nobody connected the firmware's output is dropped, like a USB port with no host."""

from __future__ import annotations

import socket


class McuPort:
    def __init__(self, port: int = 5761, log=None):
        self.log = log or (lambda m: None)
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port))
        self.srv.listen(1)
        self.srv.setblocking(False)
        self.conn = None
        self.out = bytearray()
        self.port = port
        self.rx_bytes = self.tx_bytes = 0

    def exchange(self, fw):
        if self.conn is None:
            try:
                self.conn, _ = self.srv.accept()
                self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.conn.setblocking(False)
                self.out.clear()
                self.log(f"MCU port {self.port}: the Jetson side connected")
            except BlockingIOError:
                fw.tx()                                    # no host: drop what the firmware sent
                return
        try:
            while True:
                d = self.conn.recv(65536)
                if not d:
                    raise ConnectionError("closed")
                self.rx_bytes += len(d)
                fw.rx(d)
        except BlockingIOError:
            pass
        except (ConnectionError, OSError):
            self.log(f"MCU port {self.port}: the Jetson side disconnected")
            self.conn.close()
            self.conn = None
            return
        self.out += fw.tx()
        if self.out:
            try:
                n = self.conn.send(self.out)
                self.tx_bytes += n
                del self.out[:n]
            except BlockingIOError:
                pass
            except OSError:
                self.conn.close()
                self.conn = None
            if len(self.out) > 1 << 20:                    # the host stopped reading
                self.out.clear()

    def close(self):
        if self.conn is not None:
            self.conn.close()
        self.srv.close()
