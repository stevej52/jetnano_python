"""Turns control frames into servo angles on the PCA9685.

The maths is kept apart from the hardware so it can be tested: :class:`Drive`
computes angles and hands them to a :class:`ServoOutput`, which is either the
real :class:`PCA9685Output` or a :class:`MockServoOutput`.

Channel layout, as on the original robot: channel 0 is the throttle ESC,
channel 1 the front steering servo and channel 2 the rear steering servo,
which turns the opposite way for four-wheel steering. The default angles are
the ones from the ROS 2 node; they will need retuning on the real car.
"""
from __future__ import annotations

import abc
import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

from ._util import LogThrottle
from .protocol import ControlFrame, clamp

log = logging.getLogger("jetnano.drive")


@dataclass(frozen=True)
class ServoCalibration:
    channel: int
    center: float
    minimum: float
    maximum: float
    reverse: bool = False

    def __post_init__(self) -> None:
        if not 0 <= self.channel <= 15:
            raise ValueError(f"servo channel {self.channel} not in 0..15")
        if not 0 <= self.minimum <= self.center <= self.maximum <= 180:
            raise ValueError(f"channel {self.channel}: need 0 <= minimum <= center <= maximum <= 180")

    def angle_for(self, value: float) -> float:
        """Map -1..1 to an angle; the two halves may have different spans."""
        v = clamp(value)
        if self.reverse:
            v = -v
        span = (self.maximum - self.center) if v >= 0 else (self.center - self.minimum)
        return self.center + v * span


@dataclass(frozen=True)
class DriveCalibration:
    throttle: ServoCalibration = ServoCalibration(channel=0, center=90.0, minimum=65.0, maximum=125.0)
    steer_front: ServoCalibration = ServoCalibration(channel=1, center=85.0, minimum=30.0, maximum=135.0)
    steer_rear: ServoCalibration = ServoCalibration(channel=2, center=85.0, minimum=30.0, maximum=135.0,
                                                    reverse=True)
    four_wheel_steering: bool = True
    throttle_axis: int = 3          # index into the frame's four axes
    steer_axis: int = 2
    invert_throttle: bool = False
    invert_steer: bool = False
    throttle_limit: float = 0.6     # fraction of full throttle without turbo
    turbo_limit: float = 1.0
    throttle_slew_per_s: float = 2.0  # max increase of |throttle| per second; 0 disables
    pwm_frequency: int = 100        # the ESC zeroed properly at 100 Hz on the original
    pca_address: int = 64           # 0x40
    esc_arm_seconds: float = 1.0

    def __post_init__(self) -> None:
        for name in ("throttle_axis", "steer_axis"):
            if not 0 <= getattr(self, name) <= 3:
                raise ValueError(f"{name} must be 0..3")
        for name in ("throttle_limit", "turbo_limit"):
            if not 0 < getattr(self, name) <= 1:
                raise ValueError(f"{name} must be in (0, 1]")
        if self.throttle_slew_per_s < 0:
            raise ValueError("throttle_slew_per_s must not be negative")


@dataclass(frozen=True)
class DriveCommand:
    throttle: float = 0.0
    steer: float = 0.0
    armed: bool = False


NEUTRAL = DriveCommand()


class ServoOutput(abc.ABC):
    @abc.abstractmethod
    def set_angle(self, channel: int, angle: float) -> None: ...

    def reopen(self) -> None:
        """Re-initialise the hardware after a bus error."""

    def close(self) -> None:
        pass


class MockServoOutput(ServoOutput):
    """Records angles instead of driving hardware; can simulate bus failures."""

    def __init__(self) -> None:
        self.angles: Dict[int, float] = {}
        self.history: List[tuple] = []
        self.fail_times = 0
        self.reopens = 0
        self.closed = False

    def set_angle(self, channel: int, angle: float) -> None:
        if self.fail_times > 0:
            self.fail_times -= 1
            raise OSError("simulated I2C failure")
        self.angles[channel] = angle
        self.history.append((channel, angle))

    def reopen(self) -> None:
        self.reopens += 1

    def close(self) -> None:
        self.closed = True


class PCA9685Output(ServoOutput):
    """The real board, through Adafruit's ServoKit."""

    def __init__(self, address: int = 0x40, frequency: int = 100, channels: int = 16) -> None:
        self.address = address
        self.frequency = frequency
        self.channels = channels
        self._kit = None
        self._build()

    def _build(self) -> None:
        from adafruit_servokit import ServoKit  # imported here: only present on the robot
        try:
            kit = ServoKit(channels=self.channels, address=self.address, frequency=self.frequency)
        except TypeError:  # older servokit without the frequency argument
            kit = ServoKit(channels=self.channels, address=self.address)
            kit._pca.frequency = self.frequency
        self._kit = kit

    def set_angle(self, channel: int, angle: float) -> None:
        self._kit.servo[channel].angle = angle

    def reopen(self) -> None:
        self._kit = None
        self._build()

    def close(self) -> None:
        kit, self._kit = self._kit, None
        if kit is not None:
            try:
                kit._pca.deinit()
            except Exception:
                pass


class Drive:
    """Applies commands to the servos with a throttle ramp and an explicit neutral.

    Any frame that is not armed, or that carries the neutral or e-stop flag,
    becomes neutral. Throttle increases are ramped by ``throttle_slew_per_s``;
    decreases and stops are immediate. Servo bus errors are counted and the
    output is re-initialised after a few failures in a row, but they never
    raise into the control loop.
    """

    def __init__(self, cal: DriveCalibration, output: ServoOutput,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.cal = cal
        self.output = output
        self._clock = clock
        self._lock = threading.Lock()
        self._throttle = 0.0
        self._last_time: Optional[float] = None
        self._last_reopen = float("-inf")
        self._consecutive_errors = 0
        self._warn = LogThrottle(2.0, clock)
        self.current: DriveCommand = NEUTRAL
        self.output_errors = 0

    def startup(self) -> None:
        log.info("Arming ESC: holding neutral for %.1f s", self.cal.esc_arm_seconds)
        self.neutral()
        time.sleep(self.cal.esc_arm_seconds)

    def command_from(self, frame: ControlFrame) -> DriveCommand:
        if frame.estop or frame.neutral or not frame.armed:
            return NEUTRAL
        limit = self.cal.turbo_limit if frame.turbo else self.cal.throttle_limit
        throttle = frame.axes[self.cal.throttle_axis]
        if self.cal.invert_throttle:
            throttle = -throttle
        steer = frame.axes[self.cal.steer_axis]
        if self.cal.invert_steer:
            steer = -steer
        return DriveCommand(throttle=clamp(throttle, -limit, limit), steer=clamp(steer), armed=True)

    def apply(self, frame: ControlFrame, now: Optional[float] = None) -> DriveCommand:
        return self.apply_command(self.command_from(frame), now)

    def apply_command(self, cmd: DriveCommand, now: Optional[float] = None) -> DriveCommand:
        now = self._clock() if now is None else now
        with self._lock:
            if cmd.armed:
                throttle = self._slew(cmd.throttle, now)
                applied = DriveCommand(throttle=throttle, steer=clamp(cmd.steer), armed=True)
            else:
                self._throttle = 0.0
                self._last_time = None
                applied = NEUTRAL
            self._write(self.angles_for(applied.throttle, applied.steer), now)
            self.current = applied
            return applied

    def neutral(self) -> DriveCommand:
        return self.apply_command(NEUTRAL)

    def angles_for(self, throttle: float, steer: float) -> Dict[int, float]:
        cal = self.cal
        angles = {cal.throttle.channel: cal.throttle.angle_for(throttle),
                  cal.steer_front.channel: cal.steer_front.angle_for(steer)}
        if cal.four_wheel_steering:
            angles[cal.steer_rear.channel] = cal.steer_rear.angle_for(steer)
        return angles

    def close(self) -> None:
        try:
            self.neutral()
        finally:
            self.output.close()

    # ----- internals -----------------------------------------------------
    def _slew(self, target: float, now: float) -> float:
        rate = self.cal.throttle_slew_per_s
        dt = 0.0 if self._last_time is None else max(0.0, now - self._last_time)
        self._last_time = now
        current = self._throttle
        toward_zero = abs(target) <= abs(current) and (target == 0.0 or (target > 0) == (current > 0))
        if rate <= 0 or toward_zero:
            self._throttle = target
        else:
            step = target - current
            limit = rate * dt
            if step > limit:
                step = limit
            elif step < -limit:
                step = -limit
            self._throttle = current + step
        return self._throttle

    def _write(self, angles: Dict[int, float], now: float) -> None:
        try:
            for channel, angle in angles.items():
                self.output.set_angle(channel, angle)
            self._consecutive_errors = 0
        except Exception as exc:
            self.output_errors += 1
            self._consecutive_errors += 1
            if self._warn.allow("output"):
                log.warning("Servo output error (%d in a row): %s", self._consecutive_errors, exc)
            if self._consecutive_errors >= 3 and now - self._last_reopen >= 1.0:
                self._last_reopen = now
                try:
                    self.output.reopen()
                    log.info("Servo output re-initialised")
                except Exception as reopen_exc:
                    log.warning("Servo output re-init failed: %s", reopen_exc)
