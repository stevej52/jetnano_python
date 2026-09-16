"""Periodic health logging for the robot.

Every few seconds a snapshot of the receiver's state (active link, fail-safe
state, per-link counters), the CPU temperature and, when the sensor is
present, the BNO055 IMU readings is written as one JSON line to a rotating
log file and summarised on the console. The IMU is optional: if it is
missing or fails it is retried in the background and never stops the robot.
"""
from __future__ import annotations

import datetime as _dt
import json
import logging
import logging.handlers
import os
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

log = logging.getLogger("jetnano.telemetry")


@dataclass(frozen=True)
class TelemetryConfig:
    enabled: bool = True
    interval_s: float = 3.0
    log_path: str = "~/logs/jetnano-telemetry.log"   # empty string disables the file
    max_bytes: int = 1_000_000
    backup_count: int = 5
    imu_enabled: bool = True


def cpu_temperature_c() -> Optional[float]:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as handle:
            return int(handle.read().strip()) / 1000.0
    except (OSError, ValueError):
        return None


class Bno055Imu:
    """Lazy, self-healing wrapper around the Adafruit BNO055 driver."""

    def __init__(self, retry_s: float = 10.0, clock: Callable[[], float] = time.monotonic) -> None:
        self._retry_s = retry_s
        self._clock = clock
        self._sensor = None
        self._next_try = float("-inf")
        self._reported_missing = False
        self.available = False

    def read(self) -> Optional[dict]:
        now = self._clock()
        if self._sensor is None:
            if now < self._next_try:
                return None
            self._next_try = now + self._retry_s
            try:
                import adafruit_bno055
                import board
                import busio
                self._sensor = adafruit_bno055.BNO055_I2C(busio.I2C(board.SCL, board.SDA))
                self.available = True
                self._reported_missing = False
                log.info("BNO055 IMU online")
            except Exception as exc:
                if not self._reported_missing:
                    log.warning("BNO055 IMU unavailable, will keep retrying: %s", exc)
                    self._reported_missing = True
                return None
        try:
            sensor = self._sensor
            return {
                "temperature_c": sensor.temperature,
                "euler": sensor.euler,
                "gyro": sensor.gyro,
                "acceleration": sensor.acceleration,
                "linear_acceleration": sensor.linear_acceleration,
                "magnetic": sensor.magnetic,
                "calibration": sensor.calibration_status,
            }
        except Exception as exc:
            log.warning("BNO055 read failed, will retry: %s", exc)
            self._sensor = None
            self.available = False
            self._next_try = now + self._retry_s
            return None


class Telemetry:
    def __init__(self, cfg: TelemetryConfig, status_provider: Callable[[], dict],
                 imu: Optional[Bno055Imu] = None, clock: Callable[[], float] = time.monotonic) -> None:
        self.cfg = cfg
        self._status = status_provider
        self._imu = imu
        self._clock = clock
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._file_log: Optional[logging.Logger] = None
        self.records = 0

    def start(self) -> None:
        if self.cfg.log_path:
            path = os.path.expanduser(self.cfg.log_path)
            try:
                os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
                handler = logging.handlers.RotatingFileHandler(
                    path, maxBytes=self.cfg.max_bytes, backupCount=self.cfg.backup_count)
                handler.setFormatter(logging.Formatter("%(message)s"))
                file_log = logging.getLogger("jetnano.telemetry.file")
                file_log.propagate = False
                file_log.setLevel(logging.INFO)
                file_log.handlers.clear()
                file_log.addHandler(handler)
                self._file_log = file_log
                log.info("Telemetry log: %s", path)
            except OSError as exc:
                log.warning("Telemetry file disabled, cannot open %s: %s", path, exc)
        self._thread = threading.Thread(target=self._run, name="telemetry", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def _run(self) -> None:
        while not self._stop.wait(self.cfg.interval_s):
            try:
                self.record()
            except Exception:
                log.exception("telemetry record failed")

    def record(self) -> dict:
        snapshot = {"time": _dt.datetime.now().isoformat(timespec="seconds")}
        snapshot.update(self._status())
        snapshot["cpu_temp_c"] = cpu_temperature_c()
        if self._imu is not None:
            snapshot["imu"] = self._imu.read()
        self.records += 1
        if self._file_log is not None:
            self._file_log.info(json.dumps(snapshot, default=str))
        temp = snapshot.get("cpu_temp_c")
        log.info("status: %s cpu=%s", snapshot.get("summary", ""),
                 f"{temp:.0f}C" if temp is not None else "n/a")
        return snapshot
