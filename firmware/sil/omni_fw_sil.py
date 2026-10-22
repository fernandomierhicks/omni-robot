"""ctypes mirror of the firmware core's SIL library (design.md 30). The sim imports this; the structs must match
lib/omni_fw/hw_io.h (checked against the library's sizeof at load).

    fw = Firmware()                  # builds the library first if a source changed
    fw.inputs.t_us = ...             # fill the hardware inputs
    out = fw.tick()                  # one 1 ms tick -> FwHwOutputs
    fw.rx(b"...")                    # bytes from the Jetson
    data = fw.tx()                   # bytes to the Jetson
"""

from __future__ import annotations

import ctypes as C
import sys
from pathlib import Path

SIL = Path(__file__).resolve().parent
sys.path.insert(0, str(SIL))
import build_sil  # noqa: E402

CAN_MAX = 16


class CanFrame(C.Structure):
    _fields_ = [("id", C.c_uint32), ("len", C.c_uint8), ("data", C.c_uint8 * 8), ("_pad", C.c_uint8 * 3)]


class HwInputs(C.Structure):
    _fields_ = [
        ("t_us", C.c_uint64),
        ("n_can_rx", C.c_uint32), ("can_rx", CanFrame * CAN_MAX),
        ("imu_new", C.c_uint32), ("imu_quat", C.c_float * 4), ("imu_gyro", C.c_float * 3), ("imu_accel", C.c_float * 3),
        ("fly_new", C.c_uint32 * 2), ("fly_erpm", C.c_float * 2),
        ("feed_new", C.c_uint32), ("feed_speed", C.c_float * 2),
        ("tilt_new", C.c_uint32), ("tilt_pos", C.c_float),
        ("beam_catch", C.c_uint32), ("beam_feed_exit", C.c_uint32), ("estop", C.c_uint32), ("bumper", C.c_uint32),
        ("v_bus", C.c_float),
        ("rc_ok", C.c_uint32), ("rc", C.c_float * 8),
    ]


class HwOutputs(C.Structure):
    _fields_ = [
        ("n_can_tx", C.c_uint32), ("can_tx", CanFrame * CAN_MAX),
        ("fly_throttle", C.c_float * 2), ("fly_brake", C.c_uint32),
        ("feed_speed_cmd", C.c_float * 2),
        ("tilt_cmd", C.c_float), ("tilt_enable", C.c_uint32),
        ("buzzer", C.c_uint32), ("led", C.c_uint32),
        ("dbg_v_cmd", C.c_float * 2), ("dbg_w_cmd", C.c_float), ("dbg_wheel_cmd", C.c_float * 4),
    ]


_lib = None


def lib():
    global _lib
    if _lib is None:
        path = build_sil.build()
        L = C.CDLL(str(path))
        L.fw_create.restype = C.c_void_p
        L.fw_destroy.argtypes = [C.c_void_p]
        L.fw_tick.argtypes = [C.c_void_p, C.POINTER(HwInputs), C.POINTER(HwOutputs)]
        L.fw_rx.argtypes = [C.c_void_p, C.c_char_p, C.c_uint32]
        L.fw_tx.argtypes = [C.c_void_p, C.c_char_p, C.c_uint32]
        L.fw_tx.restype = C.c_uint32
        L.fw_mode.argtypes = [C.c_void_p]
        L.fw_faults.argtypes = [C.c_void_p]
        L.fw_odom.argtypes = [C.c_void_p, C.POINTER(C.c_float)]
        L.fw_sizeof.restype = C.c_uint32
        L.fw_frame_encode.argtypes = [C.c_uint8, C.c_uint8, C.c_char_p, C.c_uint32, C.c_char_p, C.c_uint32]
        L.fw_frame_encode.restype = C.c_uint32
        L.fw_crc16.argtypes = [C.c_char_p, C.c_uint32]
        L.fw_crc16.restype = C.c_uint32
        L.fw_dm_decode.argtypes = [C.POINTER(CanFrame), C.POINTER(C.c_float)]
        L.fw_board_report.argtypes = [C.c_char_p, C.c_uint32]
        L.fw_board_report.restype = C.c_uint32
        L.fw_param.argtypes = [C.c_char_p]
        L.fw_param.restype = C.c_double
        for k, st in enumerate((HwInputs, HwOutputs)):
            if L.fw_sizeof(k) != C.sizeof(st):
                raise RuntimeError(f"{st.__name__}: {C.sizeof(st)} bytes here, {L.fw_sizeof(k)} in hw_io.h")
        _lib = L
    return _lib


def sizeof(which: int) -> int:
    return lib().fw_sizeof(which)


def param(name: str) -> float:
    v = lib().fw_param(name.encode())
    if v < -1e299:
        raise KeyError(name)
    return v


def board_report() -> str:
    buf = C.create_string_buffer(16384)
    n = lib().fw_board_report(buf, len(buf))
    return buf.raw[:n].decode()


def frame_encode(msg_type: int, seq: int, body: bytes) -> bytes:
    out = C.create_string_buffer(512)
    n = lib().fw_frame_encode(msg_type, seq, body, len(body), out, len(out))
    return out.raw[:n]


def crc16(data: bytes) -> int:
    return lib().fw_crc16(data, len(data))


def dm_decode(frame: CanFrame):
    o = (C.c_float * 5)()
    if not lib().fw_dm_decode(C.byref(frame), o):
        return None
    return int(o[0]), int(o[1]), o[2], o[3], o[4]


class Firmware:
    """One MCU running the firmware core."""

    def __init__(self):
        self.L = lib()
        self.h = self.L.fw_create()
        self.inputs = HwInputs()
        self.outputs = HwOutputs()
        self._txbuf = C.create_string_buffer(16384)
        self._odom = (C.c_float * 9)()

    def tick(self) -> HwOutputs:
        self.L.fw_tick(self.h, C.byref(self.inputs), C.byref(self.outputs))
        return self.outputs

    def rx(self, data: bytes):
        if data:
            self.L.fw_rx(self.h, data, len(data))

    def tx(self) -> bytes:
        n = self.L.fw_tx(self.h, self._txbuf, len(self._txbuf))
        return self._txbuf.raw[:n]

    @property
    def mode(self) -> int:
        return self.L.fw_mode(self.h)

    @property
    def faults(self) -> int:
        return self.L.fw_faults(self.h)

    def odom(self):
        """(x, y, yaw, vx, vy, wz, slip, roll, pitch) of the firmware's odometry."""
        self.L.fw_odom(self.h, self._odom)
        return tuple(self._odom)

    def close(self):
        if self.h:
            self.L.fw_destroy(self.h)
            self.h = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
