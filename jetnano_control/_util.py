"""Small helpers shared by the other modules."""
from __future__ import annotations

import threading
import time
from typing import Callable, Dict


class LogThrottle:
    """Allows one event per key every ``interval_s`` seconds.

    Used to keep a persistent fault (a dead link, a failing servo bus) from
    flooding the log while still reporting it regularly.
    """

    def __init__(self, interval_s: float, clock: Callable[[], float] = time.monotonic) -> None:
        self._interval = interval_s
        self._clock = clock
        self._last: Dict[str, float] = {}
        self._lock = threading.Lock()

    def allow(self, key: str = "") -> bool:
        now = self._clock()
        with self._lock:
            last = self._last.get(key)
            if last is not None and now - last < self._interval:
                return False
            self._last[key] = now
            return True


def setup_logging(level: str = "INFO") -> None:
    import logging
    logging.basicConfig(level=getattr(logging, level.upper(), logging.INFO),
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")


def install_signal_handlers(stop: Callable[[], None]) -> None:
    """Route SIGINT and SIGTERM to ``stop`` so shutdown always runs the fail-safe path."""
    import logging
    import signal

    def handler(signum, _frame):
        logging.getLogger("jetnano").info("Signal %s received, stopping", signum)
        stop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):   # not on the main thread
            pass
