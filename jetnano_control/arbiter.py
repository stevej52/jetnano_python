"""Chooses which link's frame drives the robot.

Every link delivers the frames it decodes into :meth:`LinkArbiter.offer`.
The arbiter keeps the newest frame per link, and :meth:`LinkArbiter.select`
returns the frame from the highest-priority link that delivered within
``fresh_for`` seconds. When no link is fresh it returns ``None`` and the
caller applies the fail-safe.

Because the transmitter sends on every link at once, falling back needs no
probing: the backup link's frames were already arriving, they were simply
outranked. Recovery works the same way in reverse.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional

from .protocol import ControlFrame, seq_newer


@dataclass
class LinkRecord:
    name: str
    priority: int                       # lower number wins
    frame: Optional[ControlFrame] = None
    received_at: float = float("-inf")
    accepted: int = 0
    rejected_old: int = 0

    def age(self, now: float) -> float:
        return now - self.received_at


@dataclass(frozen=True)
class Selection:
    link: str
    frame: ControlFrame
    age: float


class LinkArbiter:
    def __init__(self, fresh_for: float, clock: Callable[[], float] = time.monotonic) -> None:
        if fresh_for <= 0:
            raise ValueError("fresh_for must be positive")
        self.fresh_for = fresh_for
        self._clock = clock
        self._lock = threading.Lock()
        self._links: Dict[str, LinkRecord] = {}

    def register(self, name: str, priority: int) -> None:
        with self._lock:
            if name in self._links:
                raise ValueError(f"link {name!r} already registered")
            self._links[name] = LinkRecord(name=name, priority=priority)

    @property
    def names(self):
        with self._lock:
            return [r.name for r in sorted(self._links.values(), key=lambda r: r.priority)]

    def offer(self, name: str, frame: ControlFrame, received_at: Optional[float] = None) -> bool:
        """Record a frame from ``name``. Returns False if it was older than the last one.

        Sequence numbers are only compared while the link is fresh; once a link
        has gone quiet for longer than ``fresh_for`` any sequence is accepted,
        so a restarted transmitter is picked up straight away.
        """
        now = self._clock() if received_at is None else received_at
        with self._lock:
            rec = self._links[name]
            if (rec.frame is not None and rec.age(now) <= self.fresh_for
                    and not seq_newer(frame.seq, rec.frame.seq)):
                rec.rejected_old += 1
                return False
            rec.frame = frame
            rec.received_at = now
            rec.accepted += 1
            return True

    def select(self, now: Optional[float] = None) -> Optional[Selection]:
        now = self._clock() if now is None else now
        with self._lock:
            for rec in sorted(self._links.values(), key=lambda r: r.priority):
                if rec.frame is not None and rec.age(now) <= self.fresh_for:
                    return Selection(rec.name, rec.frame, rec.age(now))
        return None

    def snapshot(self, now: Optional[float] = None) -> Dict[str, dict]:
        now = self._clock() if now is None else now
        with self._lock:
            out: Dict[str, dict] = {}
            for rec in sorted(self._links.values(), key=lambda r: r.priority):
                has_frame = rec.frame is not None
                age = rec.age(now) if has_frame else None
                out[rec.name] = {
                    "priority": rec.priority,
                    "accepted": rec.accepted,
                    "rejected_old": rec.rejected_old,
                    "age_s": age,
                    "fresh": has_frame and age <= self.fresh_for,
                    "seq": rec.frame.seq if has_frame else None,
                }
            return out
