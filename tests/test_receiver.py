import math

from conftest import Clock
from jetnano_control.config import Config, ReceiverConfig, TelemetryConfig
from jetnano_control.links import UdpLink
from jetnano_control.obstacle import ObstacleConfig, ObstacleGuard
from jetnano_control.protocol import FLAG_ARMED, ControlFrame
from jetnano_control.receiver import Receiver
from jetnano_control.servo import Drive, DriveCalibration, MockServoOutput


def armed(seq, steer=0.0, throttle=0.0):
    return ControlFrame(seq, (0.0, 0.0, steer, throttle), flags=FLAG_ARMED)


def make_receiver():
    cfg = Config(drive=DriveCalibration(esc_arm_seconds=0.0, throttle_slew_per_s=0.0),
                 receiver=ReceiverConfig(fresh_for_s=0.5), telemetry=TelemetryConfig(enabled=False))
    clock = Clock()
    output = MockServoOutput()
    links = [UdpLink("wifi", 1), UdpLink("radio", 3)]       # never opened; only names and priorities matter here
    receiver = Receiver(cfg, links, Drive(cfg.drive, output, clock), clock=clock)
    return receiver, output, clock


def test_step_failsafe_switching_and_recovery():
    receiver, output, clock = make_receiver()
    assert receiver.step() is None and receiver.failsafe and receiver.failsafe_entries == 0
    assert output.angles == {0: 90.0, 1: 85.0, 2: 85.0}

    receiver.offer("radio", armed(1, steer=0.5), clock())
    selection = receiver.step()
    assert selection.link == "radio" and not receiver.failsafe and receiver.active_link == "radio"
    assert output.angles[1] == 110.0 and receiver.drive.current.armed

    receiver.offer("wifi", armed(1, steer=-0.5), clock())
    assert receiver.step().link == "wifi" and receiver.switches == 1 and output.angles[1] == 57.5

    clock.advance(0.6)
    assert receiver.step() is None
    assert receiver.failsafe and receiver.failsafe_entries == 1 and receiver.active_link is None
    assert output.angles == {0: 90.0, 1: 85.0, 2: 85.0} and not receiver.drive.current.armed

    receiver.offer("wifi", armed(2), clock())
    receiver.step()
    assert not receiver.failsafe and receiver.switches == 1      # recovery is not counted as a switch

    line = receiver.status_line()
    assert "active=wifi" in line and "failsafe=0" in line and "wifi=rx2/fresh" in line
    status = receiver.status()
    assert status["links"]["wifi"]["received"] == 0 and status["arbiter"]["wifi"]["accepted"] == 2
    assert status["drive"]["armed"] and status["obstacle"] is None


def test_obstacle_guard_holds_forward_throttle_only():
    receiver, output, clock = make_receiver()
    guard = ObstacleGuard(ObstacleConfig(enabled=True, stop_distance_m=0.5, stale_after_s=1.0), clock,
                          source_factory=lambda: (lambda: None))
    receiver.obstacle = guard
    guard.update([(300, 0.2)])

    receiver.offer("wifi", armed(1, steer=0.5, throttle=0.5), clock())
    receiver.step()
    assert receiver.drive.current.throttle == 0.0 and receiver.obstacle_holds == 1
    assert output.angles[1] == 110.0                          # steering still works while held
    assert "obstacle=BLOCKED" in receiver.status_line()

    receiver.offer("wifi", armed(2, throttle=-0.5), clock())
    receiver.step()
    assert receiver.drive.current.throttle == -0.5            # reversing away is allowed

    clock.advance(2.0)                                        # camera reading went stale
    receiver.offer("wifi", armed(3, throttle=0.5), clock())
    receiver.step()
    assert receiver.drive.current.throttle == 0.5 and receiver.obstacle_holds == 1
    assert receiver.status()["obstacle"]["nearest_m"][1] == 0.2
