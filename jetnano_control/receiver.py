"""Robot side: receives frames on every link, uses the best one, drives the
servos, and stops the robot when control is lost.

Run it with ``jetnano-rx``. The control loop runs at a fixed rate and on every
tick asks the arbiter for the freshest frame; when there is none the robot
goes to neutral. The fail-safe decision is deliberately made on the same tick
as the drive command rather than in a separate watchdog thread, so a stale
command can never race past it.
"""
from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from dataclasses import replace
from typing import Callable, List, Optional

from ._util import LogThrottle, install_signal_handlers, setup_logging
from .arbiter import LinkArbiter, Selection
from .config import Config, ConfigError, load_config
from .links import Link, build_link
from .obstacle import ObstacleGuard
from .protocol import ControlFrame
from .servo import NEUTRAL, Drive, MockServoOutput, PCA9685Output
from .telemetry import Bno055Imu, Telemetry

log = logging.getLogger("jetnano.receiver")


class Receiver:
    def __init__(self, cfg: Config, links: List[Link], drive: Drive,
                 telemetry: Optional[Telemetry] = None, obstacle: Optional[ObstacleGuard] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.cfg = cfg
        self.links = list(links)
        self.drive = drive
        self.telemetry = telemetry
        self.obstacle = obstacle
        self._clock = clock
        self.arbiter = LinkArbiter(cfg.receiver.fresh_for_s, clock)
        for link in self.links:
            self.arbiter.register(link.name, link.priority)
        self.stop_event = threading.Event()
        self.failsafe = True
        self.active_link: Optional[str] = None
        self.switches = 0
        self.failsafe_entries = 0
        self.obstacle_holds = 0
        self.loop_errors = 0
        self.ticks = 0
        self.started_at: Optional[float] = None
        self._warn = LogThrottle(2.0, clock)

    # ----- status ----------------------------------------------------------
    def status(self) -> dict:
        now = self._clock()
        cmd = self.drive.current
        return {
            "summary": self.status_line(now),
            "uptime_s": None if self.started_at is None else round(now - self.started_at, 1),
            "active_link": self.active_link,
            "failsafe": self.failsafe,
            "switches": self.switches,
            "failsafe_entries": self.failsafe_entries,
            "obstacle_holds": self.obstacle_holds,
            "loop_errors": self.loop_errors,
            "drive": {"throttle": cmd.throttle, "steer": cmd.steer, "armed": cmd.armed,
                      "output_errors": self.drive.output_errors},
            "links": {link.name: link.stats.as_dict() for link in self.links},
            "arbiter": self.arbiter.snapshot(now),
            "obstacle": None if self.obstacle is None else self.obstacle.status(),
        }

    def status_line(self, now: Optional[float] = None) -> str:
        now = self._clock() if now is None else now
        cmd = self.drive.current
        parts = [f"active={self.active_link or 'none'}", f"failsafe={int(self.failsafe)}",
                 f"armed={int(cmd.armed)}", f"thr={cmd.throttle:+.2f}", f"steer={cmd.steer:+.2f}"]
        for name, rec in self.arbiter.snapshot(now).items():
            age = rec["age_s"]
            state = "fresh" if rec["fresh"] else "stale"
            parts.append(f"{name}=rx{rec['accepted']}/{state}" + ("" if age is None else f"/{age:.2f}s"))
        if self.obstacle is not None and self.obstacle.forward_blocked(now):
            parts.append("obstacle=BLOCKED")
        return " ".join(parts)

    # ----- control ---------------------------------------------------------
    def offer(self, name: str, frame: ControlFrame, when: float) -> None:
        """The sink every link delivers into (runs on the link's thread)."""
        self.arbiter.offer(name, frame, when)

    def step(self, now: Optional[float] = None) -> Optional[Selection]:
        """One control tick: pick a frame and drive, or go to neutral."""
        now = self._clock() if now is None else now
        self.ticks += 1
        selection = self.arbiter.select(now)
        if selection is None:
            if not self.failsafe:
                self.failsafe = True
                self.failsafe_entries += 1
                self.active_link = None
                log.warning("No fresh control frames on any link; holding neutral")
            self.drive.apply_command(NEUTRAL, now)
            return None
        if self.failsafe:
            self.failsafe = False
            log.info("Control active via %s", selection.link)
        elif selection.link != self.active_link:
            self.switches += 1
            log.warning("Control switched from %s to %s", self.active_link, selection.link)
        self.active_link = selection.link
        cmd = self.drive.command_from(selection.frame)
        if cmd.throttle > 0 and self.obstacle is not None and self.obstacle.forward_blocked(now):
            cmd = replace(cmd, throttle=0.0)
            self.obstacle_holds += 1
            if self._warn.allow("obstacle"):
                log.warning("Obstacle ahead; forward throttle held at zero")
        self.drive.apply_command(cmd, now)
        return selection

    def run(self) -> None:
        self.started_at = self._clock()
        try:
            for link in self.links:
                try:
                    link.open()
                except Exception as exc:
                    log.error("%s: cannot open (%s); will keep retrying in the background", link.name, exc)
                link.start_receiving(self.offer)
            self.drive.startup()
            if self.telemetry is not None:
                self.telemetry.start()
            if self.obstacle is not None:
                self.obstacle.start()
            log.info("Receiver running: links=%s rate=%.0f Hz fresh_for=%.2f s",
                     [f"{l.name}(p{l.priority})" for l in self.links],
                     self.cfg.receiver.control_rate_hz, self.cfg.receiver.fresh_for_s)
            period = 1.0 / self.cfg.receiver.control_rate_hz
            next_tick = self._clock()
            last_status = self._clock()
            while not self.stop_event.is_set():
                now = self._clock()
                try:
                    self.step(now)
                except Exception:
                    self.loop_errors += 1
                    log.exception("Control step failed; applying neutral")
                    try:
                        self.drive.neutral()
                    except Exception:
                        log.exception("Neutral failed")
                if now - last_status >= self.cfg.receiver.status_interval_s:
                    self._send_status(now)
                    last_status = now
                next_tick += period
                delay = next_tick - self._clock()
                if delay > 0:
                    self.stop_event.wait(delay)
                else:                       # fell behind: do not try to catch up
                    next_tick = self._clock()
        finally:
            self.shutdown()

    def request_stop(self) -> None:
        self.stop_event.set()

    def _send_status(self, now: float) -> None:
        text = self.status_line(now)
        for link in self.links:
            link.send_status(text)

    def shutdown(self) -> None:
        self.stop_event.set()
        log.info("Receiver stopping: neutral, then closing links")
        try:
            self.drive.neutral()
        except Exception:
            log.exception("Neutral failed during shutdown")
        for link in self.links:
            try:
                link.close()
            except Exception:
                log.exception("%s: close failed", link.name)
        if self.telemetry is not None:
            self.telemetry.stop()
        if self.obstacle is not None:
            self.obstacle.stop()
        try:
            self.drive.close()
        except Exception:
            log.exception("Drive close failed")


def build_receiver(cfg: Config, dry_run: bool = False) -> Receiver:
    links = [build_link(spec, "rx") for spec in cfg.enabled_links()]
    if dry_run:
        log.warning("Dry run: servo commands go to a mock, not to the PCA9685")
        output = MockServoOutput()
    else:
        output = PCA9685Output(address=cfg.drive.pca_address, frequency=cfg.drive.pwm_frequency)
    receiver = Receiver(cfg, links, Drive(cfg.drive, output))
    if cfg.telemetry.enabled:
        imu = Bno055Imu() if cfg.telemetry.imu_enabled and not dry_run else None
        receiver.telemetry = Telemetry(cfg.telemetry, receiver.status, imu)
    if cfg.obstacle.enabled and not dry_run:
        receiver.obstacle = ObstacleGuard(cfg.obstacle)
    return receiver


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="jetnano-rx",
        description="Robot-side receiver: drives the servos from the best available link.")
    parser.add_argument("-c", "--config", help="JSON config file; built-in defaults when omitted")
    parser.add_argument("--dry-run", action="store_true", help="use mock servos (no I2C hardware needed)")
    parser.add_argument("--log-level", default="INFO", help="DEBUG, INFO, WARNING or ERROR")
    args = parser.parse_args(argv)
    setup_logging(args.log_level)
    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        log.error("Bad configuration: %s", exc)
        return 2
    try:
        receiver = build_receiver(cfg, dry_run=args.dry_run)
    except Exception as exc:
        log.error("Cannot start: %s", exc)
        log.info("Use --dry-run to run without the servo hardware")
        return 1
    install_signal_handlers(receiver.request_stop)
    receiver.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
