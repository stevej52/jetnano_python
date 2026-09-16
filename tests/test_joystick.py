import math

import pytest

from conftest import Clock
from jetnano_control.joystick import (JoystickConfig, JoystickReader, JoystickSample, SweepJoystick,
                                      apply_deadzone, rescale_axis)
from jetnano_control.protocol import FLAG_ARMED, FLAG_ESTOP, FLAG_TURBO


class FakeJoystick:
    def __init__(self, name, axes, buttons):
        self.name = name
        self.axes = list(axes)
        self.buttons = list(buttons)
        self.broken = False

    def init(self):
        pass

    def get_name(self):
        return self.name

    def get_numaxes(self):
        return len(self.axes)

    def get_axis(self, index):
        if self.broken:
            raise RuntimeError("Invalid joystick device number")
        return self.axes[index]

    def get_numbuttons(self):
        return len(self.buttons)

    def get_button(self, index):
        return self.buttons[index]


class Event:
    def __init__(self, type_):
        self.type = type_


class FakePygame:
    JOYDEVICEREMOVED = 1541

    class _Joystick:
        def __init__(self, pg):
            self.pg = pg

        def init(self):
            pass

        def quit(self):
            pass

        def get_count(self):
            return len(self.pg.devices)

        def Joystick(self, index):
            return self.pg.devices[index]

    class _Event:
        def __init__(self, pg):
            self.pg = pg

        def pump(self):
            pass

        def get(self):
            events, self.pg.events = self.pg.events, []
            return events

    def __init__(self, devices=()):
        self.devices = list(devices)
        self.events = []
        self.joystick = self._Joystick(self)
        self.event = self._Event(self)
        self.quit_calls = 0

    def init(self):
        pass

    def quit(self):
        self.quit_calls += 1


def test_apply_deadzone():
    assert apply_deadzone(0.03, 0.05) == 0.0
    assert apply_deadzone(-0.05, 0.05) == 0.0
    assert apply_deadzone(1.0, 0.05) == 1.0
    assert apply_deadzone(0.525, 0.05) == pytest.approx(0.5)
    assert apply_deadzone(-0.525, 0.05) == pytest.approx(-0.5)
    assert apply_deadzone(0.3, 0.0) == 0.3
    assert apply_deadzone(4.0, 0.1) == 1.0


def test_rescale_axis():
    assert rescale_axis(0.2, 0.2) == 0.0
    assert rescale_axis(1.0, 0.2) == 1.0
    assert rescale_axis(-1.0, 0.2) == -1.0
    assert rescale_axis(0.6, 0.2) == pytest.approx(0.5)
    assert rescale_axis(-0.4, 0.2) == pytest.approx(-0.5)


def test_config_validation():
    with pytest.raises(ValueError):
        JoystickConfig(axes=(0, 1))
    with pytest.raises(ValueError):
        JoystickConfig(deadzone=0.95)


def test_reader_calibrates_rest_and_reads_buttons():
    device = FakeJoystick("Thrustmaster", axes=[0.1, -0.2, 0.0, 0.0], buttons=[1, 0, 0])
    pg = FakePygame([device])
    reader = JoystickReader(JoystickConfig(deadzone=0.0, estop_button=1, turbo_button=2), pg, Clock())
    reader.open()
    assert reader.attached and reader.name == "Thrustmaster" and reader.attach_count == 1
    sample = reader.read()
    assert sample.axes == (0.0, 0.0, 0.0, 0.0)          # at rest reads zero after calibration
    assert sample.armed and not sample.estop and not sample.turbo and sample.buttons == 0b001
    device.axes[2] = 0.5
    device.buttons[0] = 0
    device.buttons[2] = 1
    sample = reader.read()
    assert sample.axes[2] == 0.5 and not sample.armed and sample.turbo and sample.buttons == 0b100
    frame = sample.to_frame(42)
    assert frame.seq == 42 and frame.turbo and not frame.armed and frame.axes[2] == 0.5


def test_reader_inverts_axes_and_can_be_always_armed():
    device = FakeJoystick("pad", axes=[0.0, 0.0, 0.0, 0.5], buttons=[])
    reader = JoystickReader(JoystickConfig(deadzone=0.0, calibrate_rest=False, arm_button=-1,
                                           invert=(False, False, False, True)), FakePygame([device]), Clock())
    reader.open()
    sample = reader.read()
    assert sample.axes[3] == -0.5 and sample.armed and sample.buttons == 0


def test_reader_picks_device_by_name():
    devices = [FakeJoystick("Logitech Pad", [0, 0, 0, 0], [0]), FakeJoystick("Thrustmaster HOTAS", [0, 0, 0, 0], [0])]
    reader = JoystickReader(JoystickConfig(name_contains="thrust"), FakePygame(devices), Clock())
    reader.open()
    assert reader.name == "Thrustmaster HOTAS"


def test_reader_survives_missing_and_reappearing_device():
    pg = FakePygame([])
    clock = Clock()
    reader = JoystickReader(JoystickConfig(reattach_interval_s=1.0), pg, clock)
    reader.open()
    assert not reader.attached and reader.read() is None
    pg.devices.append(FakeJoystick("pad", [0, 0, 0, 0], [1]))
    assert reader.read() is None                       # too soon to rescan
    clock.advance(1.0)
    assert reader.read() is not None and reader.attached
    pg.events.append(Event(FakePygame.JOYDEVICEREMOVED))
    pg.devices.clear()
    assert reader.read() is None and not reader.attached
    pg.devices.append(FakeJoystick("pad", [0, 0, 0, 0], [1]))
    clock.advance(1.0)
    assert reader.read() is not None and reader.attach_count == 2


def test_reader_detaches_when_reads_fail():
    device = FakeJoystick("pad", [0, 0, 0, 0], [1])
    reader = JoystickReader(JoystickConfig(), FakePygame([device]), Clock())
    reader.open()
    assert reader.read() is not None
    device.broken = True
    assert reader.read() is None and not reader.attached
    reader.close()


def test_sample_flags():
    sample = JoystickSample((0, 0, 0, 0), 0, armed=True, estop=True, turbo=True)
    assert sample.to_frame(1).flags == FLAG_ARMED | FLAG_ESTOP | FLAG_TURBO


def test_sweep_joystick():
    clock = Clock(0.0)
    joystick = SweepJoystick(period_s=4.0, clock=clock)
    joystick.open()
    assert joystick.read().axes[2] == pytest.approx(0.0)
    clock.advance(1.0)
    sample = joystick.read()
    assert sample.axes[2] == pytest.approx(1.0) and sample.armed and sample.axes[3] == 0.0
    joystick.close()
