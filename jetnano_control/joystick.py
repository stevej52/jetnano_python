"""Reads the operator's joystick with pygame, headless.

pygame normally wants a display, which is why the old scripts could not run
over SSH. Setting ``SDL_VIDEODRIVER=dummy`` before pygame initialises removes
that need. The reader calibrates each axis's resting position when the
controller attaches (sticks and throttles rarely rest at exactly zero),
applies a dead zone, and survives the controller disconnecting and
reconnecting.
"""
from __future__ import annotations

import logging
import math
import os
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from .protocol import FLAG_ARMED, FLAG_ESTOP, FLAG_TURBO, Axes, ControlFrame, clamp

log = logging.getLogger("jetnano.joystick")


@dataclass(frozen=True)
class JoystickConfig:
    name_contains: str = ""                      # pick the first joystick whose name contains this
    axes: Tuple[int, int, int, int] = (0, 1, 2, 3)   # pygame axis numbers for frame axes 0..3
    invert: Tuple[bool, bool, bool, bool] = (False, False, False, False)
    deadzone: float = 0.05
    calibrate_rest: bool = True                  # treat the position at attach time as zero
    arm_button: int = 0                          # -1 means always armed (not recommended)
    estop_button: int = -1
    turbo_button: int = -1
    reattach_interval_s: float = 1.0

    def __post_init__(self) -> None:
        if len(self.axes) != 4 or len(self.invert) != 4:
            raise ValueError("axes and invert need exactly four entries")
        if not 0 <= self.deadzone < 0.9:
            raise ValueError("deadzone must be in [0, 0.9)")


def apply_deadzone(value: float, deadzone: float) -> float:
    """Zero small inputs and rescale the rest so full deflection is still 1."""
    v = clamp(value)
    if deadzone <= 0:
        return v
    if abs(v) <= deadzone:
        return 0.0
    scaled = (abs(v) - deadzone) / (1.0 - deadzone)
    return scaled if v > 0 else -scaled


def rescale_axis(value: float, rest: float) -> float:
    """Map a raw axis so that ``rest`` reads 0 while -1 and +1 stay at the ends."""
    v = clamp(value)
    rest = clamp(rest, -0.9, 0.9)
    if v < rest:
        return (v - rest) / (1.0 + rest)
    return (v - rest) / (1.0 - rest)


@dataclass(frozen=True)
class JoystickSample:
    axes: Axes
    buttons: int
    armed: bool
    estop: bool
    turbo: bool

    def to_frame(self, seq: int) -> ControlFrame:
        flags = (FLAG_ARMED if self.armed else 0) | (FLAG_ESTOP if self.estop else 0) | \
                (FLAG_TURBO if self.turbo else 0)
        return ControlFrame(seq=seq, axes=self.axes, buttons=self.buttons, flags=flags)


class JoystickReader:
    """Reads one joystick; ``read()`` returns None while no controller is attached."""

    def __init__(self, cfg: JoystickConfig, pygame_module=None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.cfg = cfg
        self._pg = pygame_module
        self._clock = clock
        self._joy = None
        self._rest = (0.0, 0.0, 0.0, 0.0)
        self._last_attach_try = float("-inf")
        self.name = ""
        self.attach_count = 0

    @property
    def attached(self) -> bool:
        return self._joy is not None

    def open(self) -> None:
        if self._pg is None:
            os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
            os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
            import pygame
            self._pg = pygame
        self._pg.init()
        self._pg.joystick.init()
        self._try_attach()

    def read(self) -> Optional[JoystickSample]:
        pg = self._pg
        try:
            pg.event.pump()
            removed = getattr(pg, "JOYDEVICEREMOVED", None)
            for event in pg.event.get():
                if removed is not None and event.type == removed:
                    self._detach("device removed")
            if self._joy is None:
                if self._clock() - self._last_attach_try >= self.cfg.reattach_interval_s:
                    self._try_attach()
                if self._joy is None:
                    return None
            joy = self._joy
            axis_count = joy.get_numaxes()
            axes = []
            for index, rest, invert in zip(self.cfg.axes, self._rest, self.cfg.invert):
                raw = joy.get_axis(index) if index < axis_count else 0.0
                value = apply_deadzone(rescale_axis(raw, rest), self.cfg.deadzone)
                axes.append(-value if invert else value)
            button_count = joy.get_numbuttons()
            mask = 0
            for button in range(min(button_count, 16)):
                if joy.get_button(button):
                    mask |= 1 << button

            def pressed(index: int) -> bool:
                return 0 <= index < button_count and bool(joy.get_button(index))

            armed = self.cfg.arm_button < 0 or pressed(self.cfg.arm_button)
            return JoystickSample(tuple(axes), mask, armed,
                                  pressed(self.cfg.estop_button), pressed(self.cfg.turbo_button))
        except Exception as exc:
            self._detach(f"read failed: {exc}")
            return None

    def close(self) -> None:
        pg = self._pg
        if pg is None:
            return
        try:
            pg.joystick.quit()
            pg.quit()
        except Exception:
            pass

    # ----- internals -----------------------------------------------------
    def _try_attach(self) -> bool:
        pg = self._pg
        self._last_attach_try = self._clock()
        try:
            pg.joystick.quit()   # rescan so a controller plugged in later is seen
            pg.joystick.init()
            pg.event.pump()
            for index in range(pg.joystick.get_count()):
                joy = pg.joystick.Joystick(index)
                joy.init()
                name = joy.get_name()
                if self.cfg.name_contains and self.cfg.name_contains.lower() not in name.lower():
                    continue
                self._joy = joy
                self.name = name
                self._rest = self._read_rest(joy) if self.cfg.calibrate_rest else (0.0, 0.0, 0.0, 0.0)
                self.attach_count += 1
                log.info("Joystick attached: %s (%d axes, %d buttons), rest positions %s", name,
                         joy.get_numaxes(), joy.get_numbuttons(),
                         " ".join(f"{r:+.2f}" for r in self._rest))
                return True
        except Exception as exc:
            log.warning("Joystick attach failed: %s", exc)
        self._joy = None
        return False

    def _read_rest(self, joy) -> Axes:
        pg = self._pg
        axis_count = joy.get_numaxes()
        rest = [0.0, 0.0, 0.0, 0.0]
        for _ in range(3):   # let SDL deliver the first axis values
            pg.event.pump()
            time.sleep(0.02)
            rest = [joy.get_axis(i) if i < axis_count else 0.0 for i in self.cfg.axes]
        return tuple(rest)

    def _detach(self, why: str) -> None:
        if self._joy is not None:
            log.warning("Joystick detached: %s", why)
        self._joy = None


class SweepJoystick:
    """A stand-in joystick for bench tests: slow sine on steering, armed, no throttle."""

    name = "sweep"
    attached = True

    def __init__(self, period_s: float = 4.0, clock: Callable[[], float] = time.monotonic) -> None:
        self._period = period_s
        self._clock = clock
        self._t0 = clock()

    def open(self) -> None:
        pass

    def read(self) -> Optional[JoystickSample]:
        phase = 2 * math.pi * ((self._clock() - self._t0) % self._period) / self._period
        return JoystickSample((0.0, 0.0, math.sin(phase), 0.0), buttons=1, armed=True, estop=False, turbo=False)

    def close(self) -> None:
        pass
