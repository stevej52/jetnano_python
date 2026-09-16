import math
import time

from conftest import Clock, wait_until
from jetnano_control.obstacle import ObstacleConfig, ObstacleGuard, nearest_by_band


def test_nearest_by_band_splits_columns_and_ignores_invalid():
    samples = [(100, 2.0), (200, 0.9), (250, 0.0), (300, 0.7), (330, float("nan")), (400, 0.1), (500, 1.5), (539, 1.2)]
    left, center, right = nearest_by_band(samples, (100, 540), 0.15)
    assert left == 0.9 and center == 0.7 and right == 1.2
    assert nearest_by_band([], (0, 300), 0.15) == (math.inf, math.inf, math.inf)


def test_guard_blocks_only_on_fresh_center_readings():
    clock = Clock(0.0)
    guard = ObstacleGuard(ObstacleConfig(enabled=True, stop_distance_m=0.5, stale_after_s=1.0), clock,
                          source_factory=lambda: (lambda: None))
    assert not guard.forward_blocked()
    guard.update([(300, 0.3)])
    assert guard.forward_blocked()
    clock.advance(0.5)
    assert guard.forward_blocked()
    clock.advance(1.0)
    assert not guard.forward_blocked()                  # stale reading never blocks
    guard.update([(120, 0.3), (300, 0.9)])
    assert not guard.forward_blocked()                  # something on the left only
    assert guard.status()["nearest_m"] == [0.3, 0.9, math.inf]


class FlakySource:
    made = 0

    def __init__(self):
        FlakySource.made += 1
        self.calls = 0
        self.closed = False

    def __call__(self):
        self.calls += 1
        if self.calls > 3:
            raise RuntimeError("camera dropped")
        time.sleep(0.005)
        return [(300, 0.2)]

    def close(self):
        self.closed = True


def test_guard_thread_recovers_from_camera_failures():
    FlakySource.made = 0
    guard = ObstacleGuard(ObstacleConfig(enabled=True, retry_s=0.05), source_factory=FlakySource)
    guard.start()
    try:
        assert wait_until(lambda: FlakySource.made >= 2 and guard.frames >= 4)
        assert guard.errors >= 1 and guard.forward_blocked()
    finally:
        guard.stop()
