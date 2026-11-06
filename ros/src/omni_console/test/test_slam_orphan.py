"""The console's slam_toolbox outlives a console killed hard (its own session): the next console stops it before
starting its own (mission_node.SlamSupervisor._reap_orphan; found Nov 5 when a restarted console localized 0.9 m off)."""

import os
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(sys.platform == "win32", reason="process groups and /proc: Linux (WSL, the robot)")


def supervisor(tmp_path):
    mn = pytest.importorskip("omni_console.mission_node")
    tpl = tmp_path / "loc.yaml"
    tpl.write_text("")
    return mn.SlamSupervisor("m.yaml", str(tpl), tmp_path, True, tmp_path, log=lambda m: None)


def group(name: str) -> subprocess.Popen:
    """A process group whose member's command line says `name`, as an orphaned `ros2 launch slam_toolbox` would."""
    return subprocess.Popen(["bash", "-c", f"exec -a {name} sleep 60"], start_new_session=True)


def test_an_orphaned_slam_toolbox_is_stopped(tmp_path):
    s = supervisor(tmp_path)
    p = group("localization_slam_toolbox_node")
    time.sleep(0.2)
    (tmp_path / "slam_toolbox.pgid").write_text(str(p.pid))
    s._reap_orphan()
    assert p.wait(5) is not None
    assert not (tmp_path / "slam_toolbox.pgid").exists()


def test_a_stale_pgid_of_something_else_is_left_alone(tmp_path):
    s = supervisor(tmp_path)
    p = group("not_slam")                       # the pid was reused by an unrelated process
    time.sleep(0.2)
    (tmp_path / "slam_toolbox.pgid").write_text(str(p.pid))
    s._reap_orphan()
    assert p.poll() is None
    os.killpg(p.pid, 9)
    p.wait(5)
