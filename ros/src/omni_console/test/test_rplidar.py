"""rplidar, the Lidar page's driver: its three answer formats decoded from a real A2M8's bytes (test/data, recorded
Nov 11 on the Pi, the lidar still on a desk), the device protocol against FakeSerial, and camera_snap's /lidar routes.
Run from the repo root:
  & $py -m pytest -q ros/src/omni_console/test
"""

import json
import os
import statistics
import struct
import sys
import threading
import time
import urllib.error
import urllib.request

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omni_console import camera_snap as cs                                # noqa: E402
from omni_console import rplidar as rp                                    # noqa: E402

DATA = os.path.join(os.path.dirname(__file__), "data", "rplidar_a2m8")
FORMATS = {"standard": rp.ANS_STANDARD, "express": rp.ANS_EXPRESS, "boost": rp.ANS_ULTRA, "stability": rp.ANS_ULTRA}


def recorded(name):
    with open(os.path.join(DATA, f"{name}.bin"), "rb") as f:
        return rp.decode_bytes(FORMATS[name], f.read())


def sectors(turn, width=15):
    s = {}
    for a, r in zip(turn.angle, turn.dist):
        if r > 0:
            s.setdefault(int(a // width), []).append(r)
    return {k: statistics.median(v) for k, v in s.items()}


def test_model_names():
    assert rp.model_name(0x28) == "A2M8" and rp.model_name(0x2C) == "A2M12" and rp.model_name(0x61) == "model 0x61"
    assert rp.request(rp.SET_PWM, b"\x94\x02") == bytes((0xA5, 0xF0, 2, 0x94, 0x02, 0xA5 ^ 0xF0 ^ 2 ^ 0x94 ^ 0x02))


@pytest.mark.parametrize("name, per_turn", [("standard", 164), ("express", 329), ("boost", 659), ("stability", 329)])
def test_recorded_modes_decode_to_whole_turns(name, per_turn):
    """The A2M8 at PWM 660 turned ~12.2 Hz: 2, 4, 8 and 4 k samples/s."""
    turns = recorded(name)
    assert len(turns) >= 2
    for t in turns:
        assert abs(len(t.angle) - per_turn) <= 2
        assert all(0 <= a < 360 for a in t.angle)
        assert 0.3 < t.stats()["valid"] <= 1.0


@pytest.mark.parametrize("name", ["express", "boost", "stability"])
def test_recorded_modes_see_the_same_room_as_standard(name):
    """Same desk, same seconds: each 15 deg sector's median range within a few % of the standard scan's (the capsule
    formats' angle offsets and range predictions decoded right)."""
    ref, got = sectors(recorded("standard")[0]), sectors(recorded(name)[0])
    common = set(ref) & set(got)
    assert len(common) >= 15
    rel = [abs(got[k] - ref[k]) / ref[k] for k in common]
    assert statistics.median(rel) < 0.03, rel


def test_parsers_skip_garbage_and_count_it():
    with open(os.path.join(DATA, "express.bin"), "rb") as f:
        data = f.read()
    p = rp.ExpressParser()
    n_clean = len(rp.decode_bytes(rp.ANS_EXPRESS, data))
    out = p.feed(b"\x01\x02\x03" + data[:500]) + p.feed(data[500:])          # junk first, split mid-capsule
    asm, turns = None, []
    asm = rp.TurnAssembler(turns.append)
    asm.add(out, 0.0)
    assert p.bad >= 3 and len(turns) == n_clean
    s = rp.StandardParser()
    assert s.feed(b"\x00\x00\x00\x00\x00") == [] and s.bad > 0                # S == !S: not a sample


# -- the device against FakeSerial ----------------------------------------------------------------------------------
def fake_lidar(answer_baud=115200):
    return rp.RPLidar("fake", opener=lambda p, b: rp.FakeSerial(p, b, answer_baud=answer_baud))


@pytest.mark.parametrize("baud", [115200, 256000])
def test_finds_the_baud_and_reads_info_health_and_modes(baud):
    lid = fake_lidar(baud)
    try:
        assert lid.baud == baud and lid.info["name"] == "A2M8" and lid.info["firmware"] == "1.26"
        assert lid.info["hardware"] == 5 and len(lid.info["serial"]) == 32
        assert lid.health == dict(status="good", code=0) and lid.rates == dict(standard_us=500, express_us=250)
        assert [(m["name"], m["rate"], m["typical"]) for m in lid.modes] == [("Standard", 2000, True), ("Express", 4000, False)]
        assert lid.pwm == 0 and not lid.ser.dtr                                # motor off until asked (A2: DTR
        #                                                                        cleared, else it runs: rplidar.py top)
    finally:
        lid.close()


def test_no_answer_is_said_plainly():
    with pytest.raises(rp.LidarError, match="no RPLIDAR answer"):
        rp.RPLidar("fake", opener=lambda p, b: rp.FakeSerial(p, b, answer_baud=9600))


@pytest.mark.parametrize("mode", ["Standard", "Express"])
def test_scans_the_fake_room(mode):
    lid = fake_lidar()
    try:
        lid.start(mode, spin_up_s=0.0)
        assert not lid.ser.dtr and lid.pwm == rp.DEFAULT_PWM and lid.running == mode
        t = lid.wait(0, 3.0)
        t = lid.wait(t.seq, 3.0)
        assert t is not None
        want = 200 if mode == "Standard" else 400                             # 2 / 4 k per s at 10 Hz
        assert abs(len(t.angle) - want) <= 40
        for a, r in zip(t.angle[::10], t.dist[::10]):                         # the room's walls and post, to the mm
            assert r == pytest.approx(rp.FakeSerial.range_mm(a), abs=25 if mode == "Express" else 2)
        time.sleep(0.4)
        assert lid.rate()["hz"] == pytest.approx(10.0, rel=0.25)
        lid.set_pwm(330)
        assert lid.ser.pwm == 330
    finally:
        lid.close()
    assert lid.ser is None


def test_stop_turns_the_motor_off_and_health_can_be_read_again():
    lid = fake_lidar()
    try:
        lid.start("Standard", spin_up_s=0.0)
        assert lid.wait(0, 3.0) is not None
        lid.stop()
        assert lid.running is None and lid.pwm == 0 and not lid.ser.dtr and lid.ser.scanning is None
        k0 = lid.ser.k
        lid.ser.write(rp.request(rp.SCAN))                                     # a raw scan: no samples, the motor is off
        time.sleep(0.2)
        lid.ser._generate()
        assert lid.ser.k == k0
        lid.ser.write(rp.request(rp.STOP))
        lid._flush()
        assert lid.refresh_health()["status"] == "good"
        with pytest.raises(rp.LidarError, match="no scan mode"):
            lid.start("Warp")
    finally:
        lid.close()


# -- camera_snap's /lidar --------------------------------------------------------------------------------------------
def get(url):
    with urllib.request.urlopen(url, timeout=8) as r:
        return json.loads(r.read())


@pytest.fixture
def server():
    lidar = rp.LidarSession(idle_s=0.8, fake=True)
    httpd = cs.serve({}, port=0, host="127.0.0.1", lidar=lidar)
    yield f"http://127.0.0.1:{httpd.server_address[1]}", lidar
    httpd.shutdown()
    with lidar.lock:
        lidar.close()


def test_lidar_routes(server):
    base, lidar = server
    d = get(base + "/lidar")
    assert d["open"] and d["info"]["name"] == "A2M8" and d["health"]["status"] == "good" and not d["running"]
    with pytest.raises(urllib.error.HTTPError) as e:
        get(base + "/lidar/scan")
    assert e.value.code == 503 and "Start" in json.loads(e.value.read())["error"]
    d = get(base + "/lidar/set?run=1&mode=Standard&pwm=660")
    assert d["running"] == "Standard" and d["pwm"] == 660
    s = get(base + "/lidar/scan")
    s2 = get(f"{base}/lidar/scan?after={s['seq']}")
    assert s2["seq"] > s["seq"] and len(s2["a"]) == len(s2["r"]) == len(s2["q"]) > 150
    assert all(0 <= a < 36000 for a in s2["a"]) and s2["stats"]["valid"] > 0.9
    assert get(base + "/lidar/set?mode=Express")["running"] == "Express"     # a mode change while running restarts it
    s3 = get(f"{base}/lidar/scan?after={s2['seq']}")                         # and the turn numbers carry on
    assert s3["seq"] > s2["seq"] and s3["mode"] == "Express"
    assert get(base + "/lidar/set?pwm=500")["pwm"] == 500
    d = get(base + "/lidar/set?run=0&health=1")
    assert d["running"] is None and d["pwm"] == 0


def test_a_scan_request_waits_across_a_mode_change(server):
    """The page's long poll for the next turn must not fail while a mode change restarts the scan (a 503 each switch,
    Nov 12 on the Pi): it gets the new mode's first turn."""
    base, lidar = server
    lidar.idle_s = 30.0
    get(base + "/lidar/set?run=1&mode=Standard")
    s = get(base + "/lidar/scan")
    d = lidar.dev
    d.ser.read = lambda n=1: (time.sleep(0.05), b"")[1]     # no turns: the poll below is still waiting at the switch
    stop = d.stop

    def stop_then_talk():
        stop()
        del d.ser.read                                       # the restarted scan gets data again
    d.stop = stop_then_talk
    out, err = {}, []

    def poll():
        try:
            out.update(get(f"{base}/lidar/scan?after={s['seq']}&timeout=4"))
        except urllib.error.HTTPError as e:
            err.append(f"{e.code} {e.read()}")
    th = threading.Thread(target=poll)
    th.start()
    time.sleep(0.3)
    assert get(base + "/lidar/set?mode=Express")["running"] == "Express"
    th.join(6)
    assert not err and out["seq"] > s["seq"] and out["mode"] == "Express"


def test_a_short_pause_in_the_data_is_not_called_no_data():
    lid = fake_lidar()
    try:
        lid.start("Standard", spin_up_s=0.0)
        assert lid.wait(0, 3.0) is not None
        read = lid.ser.read
        quiet_until = [time.monotonic() + 1.0]
        lid.ser.read = lambda n=1: (time.sleep(0.3), b"")[1] if time.monotonic() < quiet_until[0] else read(n)
        time.sleep(0.7)
        assert lid.error is None                                             # 0.7 s quiet: Boost's pause after a switch
        time.sleep(0.6)
        quiet_until[0] = time.monotonic() + 1.6
        time.sleep(1.4)
        assert lid.error and "no data" in lid.error                          # over NO_DATA_S: said
    finally:
        lid.close()


def test_lidar_closes_when_idle_and_the_motor_stops(server):
    base, lidar = server
    get(base + "/lidar/set?run=1&mode=Standard")
    ser = lidar.dev.ser
    t_end = time.time() + 5
    while not ser.closed and time.time() < t_end:
        time.sleep(0.1)
    assert lidar.dev is None and ser.closed and not ser.dtr and ser.pwm == 0


def test_spin_motor_alone_then_scan(server):
    base, lidar = server
    d = get(base + "/lidar/set?spin=1&pwm=500")
    assert d["spinning"] and d["running"] is None and d["pwm"] == 500
    assert not lidar.dev.ser.dtr and lidar.dev.ser.pwm == 500 and lidar.dev.ser.scanning is None
    assert get(base + "/lidar/set?pwm=300")["pwm"] == 300                  # the speed changes while it spins
    d = get(base + "/lidar/set?run=1&mode=Standard")                        # Start from spinning: a scan
    assert d["running"] == "Standard" and not d["spinning"]
    d = get(base + "/lidar/set?run=0")
    assert d["running"] is None and not d["spinning"] and d["pwm"] == 0 and not lidar.dev.ser.dtr
    texts = [e["text"] for e in d["events"]]
    assert any(t.startswith("opened A2M8") for t in texts) and "motor spinning at PWM 500, no scan" in texts
    assert "stopped (motor off)" in texts


def test_soft_start_ramps_the_pwm():
    lid = fake_lidar()
    sent = []
    write = lid.ser.write
    lid.ser.write = lambda b: (sent.append(bytes(b)), write(b))[1]
    try:
        lid.spin(True, 660, spin_up_s=0.3)
        pwms = [struct.unpack("<H", b[3:5])[0] for b in sent if b[1] == rp.SET_PWM]
        assert pwms[0] == rp.SOFT_START_PWM and pwms[-1] == 660 and pwms == sorted(pwms) and len(pwms) >= 3
    finally:
        lid.close()


def test_a_usb_drop_while_idle_is_retried_on_the_next_request(server):
    """The adapter reset since the last request (a stale handle, Errno 5 on write): the request reopens and works."""
    base, lidar = server
    get(base + "/lidar")
    lidar.dev.ser.unplugged = True
    d = get(base + "/lidar/set?run=1&mode=Standard")
    assert d["running"] == "Standard" and not lidar.dev.ser.unplugged
    lost = [e for e in d["events"] if e["text"].startswith("lost the lidar")]
    assert len(lost) == 1 and lost[0]["level"] == "error" and "Input/output error" in lost[0]["text"]


def test_a_usb_drop_while_scanning_is_reported_with_the_likely_cause(server):
    base, lidar = server
    lidar.idle_s = 30.0                                                    # the idle closer mustn't get there first
    get(base + "/lidar/set?run=1&mode=Standard")
    lidar.dev.ser.unplugged = True                                         # right after the spin-up: a 5 V dip?
    t_end = time.time() + 3
    while lidar.dev is not None and lidar.dev.dead is None and time.time() < t_end:
        time.sleep(0.05)
    d = get(base + "/lidar")
    assert d["open"] and d["running"] is None                               # opened again, motor off
    lost = [e["text"] for e in d["events"] if e["text"].startswith("lost the lidar")]
    assert lost and "while scanning (Standard)" in lost[-1] and "after the motor started" in lost[-1]
    assert "5 V" in lost[-1]


def test_reconnect_route(server):
    base, lidar = server
    first = get(base + "/lidar")
    ser = lidar.dev.ser
    d = get(base + "/lidar/set?reopen=1")
    assert d["open"] and ser.closed and lidar.dev.ser is not ser
    texts = [e["text"] for e in d["events"]]
    assert "reconnect asked" in texts and sum(t.startswith("opened") for t in texts) == 2
    assert first["info"] == d["info"]


def test_probe_hints():
    assert "held low" in rp.probe_hint([b"\x00" * 7, b"\x00" * 7])
    assert "nothing came back" in rp.probe_hint([b"", b""])
    assert rp.probe_hint([b"\x12\x34", b""]) == ""
    with pytest.raises(rp.LidarError, match="held low"):
        class Zeros(rp.FakeSerial):
            def read(self, n=1):
                return b"\x00" * n
        rp.RPLidar("fake", opener=lambda p, b: Zeros(p, b))


def test_no_port_says_so():
    lidar = rp.LidarSession(port=None)
    lidar.port_arg = None
    rp_find, rp.find_port = rp.find_port, lambda: None
    try:
        d = lidar.describe()
        assert not d["open"] and "plugged in" in d["error"]
    finally:
        rp.find_port = rp_find


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
