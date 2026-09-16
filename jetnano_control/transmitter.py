"""Operator side: reads the joystick and sends every frame on every link.

Run it with ``jetnano-tx``. There is no link selection here at all. Each link
has its own sender thread that forwards the newest frame at that link's own
rate, so a slow radio or a hung TCP write never delays the others, and the
robot decides which link to trust. Status lines the robot sends back over
UDP are printed so the operator can see which link is actually in control.
"""
from __future__ import annotations

import argparse
import logging
import random
import sys
import threading
import time
from dataclasses import replace
from typing import Callable, List, Optional

from ._util import install_signal_handlers, setup_logging
from .config import Config, ConfigError, load_config
from .joystick import JoystickReader, SweepJoystick
from .links import Link, UdpLink, build_link
from .protocol import FLAG_ESTOP, FLAG_NEUTRAL, ControlFrame

log = logging.getLogger("jetnano.transmitter")


class Transmitter:
    def __init__(self, cfg: Config, links: List[Link], joystick,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.cfg = cfg
        self.links = list(links)
        self.joystick = joystick
        self._clock = clock
        self.stop_event = threading.Event()
        self.seq = random.randrange(0x10000)
        self.frames = 0
        self.robot_status = ""
        self.robot_status_at = float("-inf")
        self._robot_active: Optional[str] = None
        self._had_joystick = False
        self._lock = threading.Lock()

    def next_frame(self) -> ControlFrame:
        sample = self.joystick.read()
        if sample is None:
            if self._had_joystick:
                log.warning("Joystick lost; sending neutral until it returns")
                self._had_joystick = False
            frame = ControlFrame.neutral_frame(self.seq)
        else:
            if not self._had_joystick:
                log.info("Joystick active: %s", getattr(self.joystick, "name", "?"))
                self._had_joystick = True
            frame = sample.to_frame(self.seq)
        self.seq = (self.seq + 1) & 0xFFFF
        self.frames += 1
        return frame

    def run(self) -> None:
        try:
            self.joystick.open()
            for link in self.links:
                link.on_status = self._on_status
                try:
                    link.open()
                except Exception as exc:
                    log.error("%s: cannot open (%s); will keep retrying in the background", link.name, exc)
                link.start_sending()
                if isinstance(link, UdpLink):
                    link.start_receiving(self._ignore_frame)   # only for status lines
            log.info("Transmitter running: links=%s rate=%.0f Hz",
                     [link.name for link in self.links], self.cfg.transmitter.rate_hz)
            period = 1.0 / self.cfg.transmitter.rate_hz
            next_tick = self._clock()
            last_print = self._clock()
            while not self.stop_event.is_set():
                frame = self.next_frame()
                for link in self.links:
                    link.offer_to_send(frame)
                now = self._clock()
                if now - last_print >= self.cfg.transmitter.status_print_interval_s:
                    log.info(self.status_line())
                    last_print = now
                next_tick += period
                delay = next_tick - self._clock()
                if delay > 0:
                    self.stop_event.wait(delay)
                else:
                    next_tick = self._clock()
        finally:
            self.shutdown()

    def request_stop(self) -> None:
        self.stop_event.set()

    def status_line(self) -> str:
        now = self._clock()
        joystick = "ok" if getattr(self.joystick, "attached", True) else "MISSING"
        links = " ".join(f"{link.name}=tx{link.stats.sent}/err{link.stats.send_errors}" for link in self.links)
        if self.robot_status_at == float("-inf"):
            robot = "robot: no status received yet"
        else:
            robot = f"robot ({now - self.robot_status_at:.1f}s ago): {self.robot_status}"
        return f"joystick={joystick} seq={self.seq} {links} | {robot}"

    def shutdown(self) -> None:
        self.stop_event.set()
        for link in self.links:
            link.stop_workers()
        count = self.cfg.transmitter.shutdown_frames
        if count:
            log.info("Transmitter stopping: sending e-stop on all links")
            for offset in range(count):
                frame = ControlFrame.neutral_frame((self.seq + offset) & 0xFFFF, FLAG_NEUTRAL | FLAG_ESTOP)
                for link in self.links:
                    link.send_now(frame)
                time.sleep(0.02)
            self.seq = (self.seq + count) & 0xFFFF
        for link in self.links:
            try:
                link.close()
            except Exception:
                log.exception("%s: close failed", link.name)
        self.joystick.close()

    # ----- internals -------------------------------------------------------
    def _ignore_frame(self, name: str, frame: ControlFrame, when: float) -> None:
        pass

    def _on_status(self, text: str) -> None:
        active = None
        for token in text.split():
            if token.startswith("active="):
                active = token[len("active="):]
                break
        with self._lock:
            previous = self._robot_active
            self._robot_active = active
            self.robot_status = text
            self.robot_status_at = self._clock()
        if active != previous:
            if active in (None, "none"):
                log.warning("Robot reports it has NO control link")
            else:
                log.info("Robot reports control via %s", active)


def build_transmitter(cfg: Config, fake_joystick: bool = False) -> Transmitter:
    links = [build_link(spec, "tx") for spec in cfg.enabled_links()]
    joystick = SweepJoystick() if fake_joystick else JoystickReader(cfg.joystick)
    if fake_joystick:
        log.warning("Fake joystick: sending a slow steering sweep with zero throttle")
    return Transmitter(cfg, links, joystick)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="jetnano-tx",
        description="Operator-side transmitter: sends joystick frames on every link.")
    parser.add_argument("-c", "--config", help="JSON config file; built-in defaults when omitted")
    parser.add_argument("--robot-host", help="override the robot address of the UDP and Modbus links")
    parser.add_argument("--fake-joystick", action="store_true",
                        help="send a slow steering sweep instead of reading a joystick (bench test)")
    parser.add_argument("--log-level", default="INFO", help="DEBUG, INFO, WARNING or ERROR")
    args = parser.parse_args(argv)
    setup_logging(args.log_level)
    try:
        cfg = load_config(args.config)
        if args.robot_host:
            cfg = replace(cfg, links=tuple(
                replace(spec, robot_host=args.robot_host) if spec.type in ("udp", "modbus") else spec
                for spec in cfg.links))
    except ConfigError as exc:
        log.error("Bad configuration: %s", exc)
        return 2
    try:
        transmitter = build_transmitter(cfg, fake_joystick=args.fake_joystick)
    except Exception as exc:
        log.error("Cannot start: %s", exc)
        return 1
    install_signal_handlers(transmitter.request_stop)
    transmitter.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
