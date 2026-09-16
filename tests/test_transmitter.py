import threading
import time
from dataclasses import replace

from conftest import wait_until
from jetnano_control.config import Config, TelemetryConfig, TransmitterConfig
from jetnano_control.joystick import JoystickSample
from jetnano_control.links import Link
from jetnano_control.transmitter import Transmitter


class ScriptedJoystick:
    name = "scripted"

    def __init__(self):
        self.sample = JoystickSample((0.0, 0.0, 0.5, 0.2), 1, armed=True, estop=False, turbo=False)
        self.attached = True
        self.opened = False
        self.closed = False
        self._lock = threading.Lock()

    def open(self):
        self.opened = True

    def read(self):
        with self._lock:
            return self.sample if self.attached else None

    def set(self, **changes):
        with self._lock:
            self.sample = replace(self.sample, **changes)

    def close(self):
        self.closed = True


class RecordingLink(Link):
    def __init__(self, name, priority, max_send_hz=0.0):
        super().__init__(name, priority, max_send_hz)
        self.sent = []

    def _open(self):
        pass

    def _close(self):
        pass

    def _send_raw(self, frame):
        self.sent.append(frame)

    def _receive_raw(self, timeout):
        time.sleep(min(timeout, 0.01))
        return None


def make():
    cfg = Config(transmitter=TransmitterConfig(rate_hz=200.0, status_print_interval_s=0.05, shutdown_frames=2),
                 telemetry=TelemetryConfig(enabled=False))
    joystick = ScriptedJoystick()
    links = [RecordingLink("a", 1), RecordingLink("b", 2)]
    return Transmitter(cfg, links, joystick), joystick, links


def test_next_frame_follows_joystick_and_sequence():
    tx, joystick, _ = make()
    first = tx.next_frame()
    assert first.armed and first.axes == (0.0, 0.0, 0.5, 0.2) and first.buttons == 1 and not first.neutral
    second = tx.next_frame()
    assert second.seq == (first.seq + 1) & 0xFFFF and tx.frames == 2
    joystick.attached = False
    lost = tx.next_frame()
    assert lost.neutral and not lost.armed and lost.axes == (0.0, 0.0, 0.0, 0.0)
    joystick.attached = True
    joystick.set(estop=True)
    assert tx.next_frame().estop


def test_status_parsing():
    tx, _, _ = make()
    assert "no status" in tx.status_line()
    tx._on_status("active=radio failsafe=0 armed=1")
    assert tx._robot_active == "radio" and "active=radio" in tx.status_line()
    tx._on_status("active=none failsafe=1")
    assert tx._robot_active == "none"


def test_run_sends_on_every_link_and_estops_on_shutdown():
    tx, joystick, links = make()
    thread = threading.Thread(target=tx.run, daemon=True)
    thread.start()
    assert wait_until(lambda: all(link.stats.sent >= 5 for link in links))
    assert joystick.opened
    last_regular = links[0].sent[-1].seq
    tx.request_stop()
    thread.join(timeout=5)
    assert not thread.is_alive()
    for link in links:
        estops = link.sent[-2:]
        assert all(frame.estop and frame.neutral and not frame.armed for frame in estops)
        assert ((estops[0].seq - last_regular) & 0xFFFF) < 0x8000        # newer than anything sent before
        assert not link.is_open
    assert joystick.closed
