"""Firmware update for a D400 camera through pyrealsense2 (README.md, Setup). Attended sessions only.

RealSense.FW.Update.exe 2.59 fails on very old firmware (5.9.x presents as "RealSense USB2", PID 0AD6, and
the tool takes it for a GMSL device: "Failed to locate a device in FW update mode"). This does the same
steps by hand: check the signed image against the camera, back the flash up (optional), switch to the
update (DFU) device, write, wait for the reboot. Oct 29, 2026: 5.9.2 -> 5.17.3.10 on serial 827312072573.

    python rs_fw_update.py Signed_Image_UVC_5_17_3_10.bin              # check only
    python rs_fw_update.py Signed_Image_UVC_5_17_3_10.bin --flash      # write it (don't unplug for 30 s after)

The image comes from https://dev.realsenseai.com/docs/firmware-releases-d400 (the zip has the .bin and the spec
update). A camera left in recovery mode (PID 0ADB / 0ADC) is restored with RealSense.FW.Update.exe -r -f <bin>.
"""
from __future__ import annotations

import argparse
import time

import rs_common as C
from rs_common import rs


def listing(ctx):
    out = []
    for d in ctx.query_devices():
        g = lambda k: d.get_info(k) if d.supports(k) else None
        out.append(dict(name=g(rs.camera_info.name), pid=g(rs.camera_info.product_id),
                        fw=g(rs.camera_info.firmware_version), update_device=d.is_update_device()))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image")
    ap.add_argument("--flash", action="store_true", help="write the image (otherwise only the compatibility check)")
    ap.add_argument("--serial")
    a = ap.parse_args()
    C.need_rs()
    image = bytearray(open(a.image, "rb").read())
    ctx = rs.context()
    dev = C.find_device(a.serial)
    print("before:", listing(ctx))
    upd = dev.as_updatable()
    ok = upd.check_firmware_compatibility(image)
    print("image compatible with this camera:", ok)
    if not ok or not a.flash:
        return
    upd.enter_update_state()
    t0, ud = time.time(), None
    while time.time() - t0 < 40 and ud is None:
        time.sleep(1)
        ud = next((d.as_update_device() for d in ctx.query_devices() if d.is_update_device()), None)
    if ud is None:
        raise SystemExit("no update device appeared; power-cycle the camera (its flash is untouched)")
    last = [-10]

    def progress(p):
        if p * 100 - last[0] >= 10:
            last[0] = p * 100
            print(f"  {p * 100:.0f}%")

    ud.update(image, progress)
    print("written; waiting 30 s for the reboot (don't unplug)")
    time.sleep(30)
    print("after:", listing(ctx))


if __name__ == "__main__":
    main()
