from dataclasses import replace

import pytest

from conftest import Clock
from jetnano_control.protocol import FLAG_ARMED, FLAG_ESTOP, FLAG_NEUTRAL, FLAG_TURBO, ControlFrame
from jetnano_control.servo import NEUTRAL, Drive, DriveCalibration, MockServoOutput, ServoCalibration


def make_drive(**overrides):
    cal = DriveCalibration(**{"throttle_slew_per_s": 0.0, **overrides})
    output = MockServoOutput()
    clock = Clock()
    return Drive(cal, output, clock), output, clock


def armed(axes, extra=0):
    return ControlFrame(1, axes, flags=FLAG_ARMED | extra)


def test_servo_calibration_mapping():
    cal = ServoCalibration(channel=1, center=85, minimum=30, maximum=135)
    assert cal.angle_for(0) == 85
    assert cal.angle_for(1) == 135 and cal.angle_for(-1) == 30
    assert cal.angle_for(0.5) == 110 and cal.angle_for(-0.5) == 57.5
    assert cal.angle_for(3) == 135 and cal.angle_for(-3) == 30
    reverse = replace(cal, reverse=True)
    assert reverse.angle_for(1) == 30 and reverse.angle_for(-1) == 135 and reverse.angle_for(0) == 85


def test_calibration_validation():
    with pytest.raises(ValueError):
        ServoCalibration(channel=0, center=50, minimum=60, maximum=100)
    with pytest.raises(ValueError):
        ServoCalibration(channel=16, center=90, minimum=0, maximum=180)
    with pytest.raises(ValueError):
        DriveCalibration(throttle_axis=4)
    with pytest.raises(ValueError):
        DriveCalibration(throttle_limit=0)
    with pytest.raises(ValueError):
        DriveCalibration(throttle_slew_per_s=-1)


def test_unarmed_neutral_and_estop_frames_go_neutral():
    drive, output, _ = make_drive()
    for flags in (0, FLAG_ESTOP | FLAG_ARMED, FLAG_NEUTRAL | FLAG_ARMED):
        cmd = drive.apply(ControlFrame(1, (0.5, 0.5, 0.5, 0.5), flags=flags))
        assert cmd == NEUTRAL
    assert output.angles == {0: 90.0, 1: 85.0, 2: 85.0}
    assert drive.current == NEUTRAL


def test_armed_frame_drives_all_three_channels_with_rear_mirror():
    drive, output, _ = make_drive()
    cmd = drive.apply(armed((0.0, 0.0, 1.0, 0.5)))    # axis 2 steer, axis 3 throttle
    assert cmd.armed and cmd.steer == 1.0 and cmd.throttle == 0.5
    assert output.angles[1] == 135.0                 # front full right
    assert output.angles[2] == 30.0                  # rear turns the other way
    assert output.angles[0] == pytest.approx(107.5)  # 90 + 0.5 * 35
    assert drive.current == cmd


def test_two_wheel_steering_leaves_rear_channel_alone():
    drive, output, _ = make_drive(four_wheel_steering=False)
    drive.apply(armed((0, 0, 1.0, 0)))
    assert set(output.angles) == {0, 1}


def test_throttle_limit_turbo_and_inversion():
    drive, _, _ = make_drive()
    assert drive.apply(armed((0, 0, 0, 1.0))).throttle == pytest.approx(0.6)
    assert drive.apply(armed((0, 0, 0, 1.0), FLAG_TURBO)).throttle == pytest.approx(1.0)
    assert drive.apply(armed((0, 0, 0, -1.0))).throttle == pytest.approx(-0.6)
    drive, _, _ = make_drive(invert_throttle=True, invert_steer=True)
    cmd = drive.apply(armed((0, 0, 0.5, 0.5)))
    assert cmd.throttle == pytest.approx(-0.5) and cmd.steer == pytest.approx(-0.5)


def test_slew_limits_throttle_increases_only():
    drive, _, clock = make_drive(throttle_slew_per_s=1.0)
    full = armed((0, 0, 0, 1.0), FLAG_TURBO)
    assert drive.apply(full, clock()).throttle == 0.0        # first armed tick: no time has passed
    clock.advance(0.25)
    assert drive.apply(full, clock()).throttle == pytest.approx(0.25)
    clock.advance(0.25)
    assert drive.apply(full, clock()).throttle == pytest.approx(0.5)
    clock.advance(0.25)
    assert drive.apply(armed((0, 0, 0, 0.1), FLAG_TURBO), clock()).throttle == pytest.approx(0.1)  # easing off is immediate
    clock.advance(0.25)
    assert drive.apply(armed((0, 0, 0, -0.6), FLAG_TURBO), clock()).throttle == pytest.approx(-0.15)  # crossing zero ramps
    clock.advance(1.0)
    assert drive.apply(armed((0, 0, 0, -0.6), FLAG_TURBO), clock()).throttle == pytest.approx(-0.6)
    assert drive.apply(ControlFrame(1, (0, 0, 0, -0.6)), clock()) == NEUTRAL   # disarm: immediate stop
    clock.advance(1.0)
    assert drive.apply(full, clock()).throttle == 0.0        # re-arming ramps from zero again


def test_output_errors_are_counted_and_output_reopened():
    drive, output, clock = make_drive()
    output.fail_times = 3
    for _ in range(3):
        drive.apply(armed((0, 0, 0, 0)), clock())
        clock.advance(0.1)
    assert drive.output_errors == 3 and output.reopens == 1
    drive.apply(armed((0, 0, 0.2, 0)), clock())
    assert drive.output_errors == 3 and output.angles[1] == pytest.approx(95.0)


def test_close_goes_neutral_and_closes_output():
    drive, output, _ = make_drive()
    drive.apply(armed((0, 0, 1.0, 0.5)))
    drive.close()
    assert output.angles == {0: 90.0, 1: 85.0, 2: 85.0} and output.closed
