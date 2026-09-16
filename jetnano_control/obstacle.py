"""Optional RealSense depth guard.

The depth image is sampled on a grid and split into a left, centre and right
band. When the nearest valid reading in the centre band is closer than
``stop_distance_m`` the receiver blocks forward throttle. The guard only ever
removes throttle, never adds it, and a stale or missing reading never blocks
driving, so a camera fault cannot strand the robot.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Tuple

from ._util import LogThrottle

log = logging.getLogger("jetnano.obstacle")

Sample = Tuple[int, float]   # (column, distance in metres)


@dataclass(frozen=True)
class ObstacleConfig:
    enabled: bool = False
    stop_distance_m: float = 0.5
    min_valid_m: float = 0.15          # readings closer than this are sensor noise
    sample_step: int = 20
    row_range: Tuple[int, int] = (100, 400)
    col_range: Tuple[int, int] = (100, 540)
    stale_after_s: float = 1.0
    retry_s: float = 10.0


@dataclass(frozen=True)
class Nearest:
    left: float
    center: float
    right: float
    at: float


def nearest_by_band(samples: Iterable[Sample], col_range: Tuple[int, int], min_valid_m: float) -> Tuple[float, float, float]:
    """Nearest valid distance in each third of the column range; ``inf`` when none."""
    left, right = col_range
    width = max(1, right - left)
    nearest = [float("inf"), float("inf"), float("inf")]
    for column, distance in samples:
        if not distance >= min_valid_m:      # also rejects NaN and 0 (no data)
            continue
        band = min(2, max(0, (column - left) * 3 // width))
        if distance < nearest[band]:
            nearest[band] = distance
    return nearest[0], nearest[1], nearest[2]


class RealSenseSource:
    """Yields depth samples from the first RealSense camera."""

    def __init__(self, cfg: ObstacleConfig) -> None:
        import pyrealsense2 as rs
        self._cfg = cfg
        self._pipeline = rs.pipeline()
        self._pipeline.start()

    def __call__(self) -> Optional[List[Sample]]:
        frames = self._pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        if not depth:
            return None
        cfg = self._cfg
        return [(x, depth.get_distance(x, y))
                for y in range(cfg.row_range[0], cfg.row_range[1], cfg.sample_step)
                for x in range(cfg.col_range[0], cfg.col_range[1], cfg.sample_step)]

    def close(self) -> None:
        try:
            self._pipeline.stop()
        except Exception:
            pass


class ObstacleGuard:
    def __init__(self, cfg: ObstacleConfig, clock: Callable[[], float] = time.monotonic,
                 source_factory: Optional[Callable[[], Callable[[], Optional[List[Sample]]]]] = None) -> None:
        self.cfg = cfg
        self._clock = clock
        self._source_factory = source_factory or (lambda: RealSenseSource(cfg))
        self._source = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._warn = LogThrottle(10.0, clock)
        self.nearest: Optional[Nearest] = None
        self.frames = 0
        self.errors = 0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="obstacle", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        source, self._source = self._source, None
        if source is not None and hasattr(source, "close"):
            source.close()

    def forward_blocked(self, now: Optional[float] = None) -> bool:
        now = self._clock() if now is None else now
        with self._lock:
            nearest = self.nearest
        if nearest is None or now - nearest.at > self.cfg.stale_after_s:
            return False
        return nearest.center < self.cfg.stop_distance_m

    def status(self) -> dict:
        with self._lock:
            nearest = self.nearest
        return {
            "camera": self._source is not None,
            "frames": self.frames,
            "errors": self.errors,
            "nearest_m": None if nearest is None else [nearest.left, nearest.center, nearest.right],
            "blocked": self.forward_blocked(),
        }

    def update(self, samples: Iterable[Sample], now: Optional[float] = None) -> Nearest:
        """Fold one depth frame into the guard (also used directly by tests)."""
        now = self._clock() if now is None else now
        left, center, right = nearest_by_band(samples, self.cfg.col_range, self.cfg.min_valid_m)
        nearest = Nearest(left, center, right, now)
        with self._lock:
            self.nearest = nearest
        self.frames += 1
        return nearest

    def _run(self) -> None:
        next_try = float("-inf")
        while not self._stop.is_set():
            if self._source is None:
                if self._clock() < next_try:
                    self._stop.wait(0.5)
                    continue
                next_try = self._clock() + self.cfg.retry_s
                try:
                    self._source = self._source_factory()
                    log.info("Depth camera online")
                except Exception as exc:
                    self.errors += 1
                    if self._warn.allow("open"):
                        log.warning("Depth camera unavailable, will retry: %s", exc)
                    continue
            try:
                samples = self._source()
            except Exception as exc:
                self.errors += 1
                if self._warn.allow("read"):
                    log.warning("Depth camera read failed, reopening: %s", exc)
                source, self._source = self._source, None
                if source is not None and hasattr(source, "close"):
                    source.close()
                continue
            if samples is not None:
                self.update(samples)
