import pytest

from conftest import Clock
from jetnano_control.arbiter import LinkArbiter
from jetnano_control.protocol import FLAG_ARMED, ControlFrame


def frame(seq):
    return ControlFrame(seq, (0.1, 0.2, 0.3, 0.4), flags=FLAG_ARMED)


def make():
    clock = Clock(100.0)
    arbiter = LinkArbiter(0.5, clock)
    arbiter.register("wifi", 1)
    arbiter.register("radio", 3)
    return arbiter, clock


def test_prefers_priority_when_both_fresh():
    arbiter, clock = make()
    assert arbiter.select() is None
    arbiter.offer("radio", frame(1))
    assert arbiter.select().link == "radio"
    arbiter.offer("wifi", frame(1))
    assert arbiter.select().link == "wifi"
    assert arbiter.names == ["wifi", "radio"]


def test_falls_back_when_primary_goes_stale_and_recovers():
    arbiter, clock = make()
    arbiter.offer("wifi", frame(1))
    arbiter.offer("radio", frame(1))
    clock.advance(0.3)
    assert arbiter.select().link == "wifi"
    clock.advance(0.3)                      # wifi is now 0.6 s old: stale
    arbiter.offer("radio", frame(2))        # radio keeps arriving
    selection = arbiter.select()
    assert selection.link == "radio" and selection.frame.seq == 2
    clock.advance(0.1)
    arbiter.offer("wifi", frame(3))         # wifi returns
    assert arbiter.select().link == "wifi"


def test_no_fresh_link_returns_none():
    arbiter, clock = make()
    arbiter.offer("wifi", frame(1))
    arbiter.offer("radio", frame(1))
    clock.advance(0.5)
    assert arbiter.select() is not None      # exactly fresh_for is still fresh
    clock.advance(0.01)
    assert arbiter.select() is None


def test_old_or_duplicate_sequence_ignored_while_fresh():
    arbiter, clock = make()
    assert arbiter.offer("wifi", frame(10))
    assert not arbiter.offer("wifi", frame(9))
    assert not arbiter.offer("wifi", frame(10))
    assert arbiter.offer("wifi", frame(11))
    snap = arbiter.snapshot()
    assert snap["wifi"]["accepted"] == 2 and snap["wifi"]["rejected_old"] == 2
    assert arbiter.select().frame.seq == 11


def test_sequence_resyncs_after_link_was_stale():
    arbiter, clock = make()
    arbiter.offer("wifi", frame(30000))
    clock.advance(1.0)
    assert arbiter.offer("wifi", frame(5))   # transmitter restarted: accepted
    assert arbiter.select().frame.seq == 5


def test_sequence_wraparound():
    arbiter, clock = make()
    assert arbiter.offer("wifi", frame(65535))
    assert arbiter.offer("wifi", frame(0))
    assert arbiter.select().frame.seq == 0


def test_snapshot_reports_freshness():
    arbiter, clock = make()
    snap = arbiter.snapshot()
    assert snap["wifi"] == {"priority": 1, "accepted": 0, "rejected_old": 0, "age_s": None, "fresh": False, "seq": None}
    arbiter.offer("radio", frame(4))
    clock.advance(0.2)
    snap = arbiter.snapshot()
    assert snap["radio"]["fresh"] and snap["radio"]["age_s"] == pytest.approx(0.2) and snap["radio"]["seq"] == 4


def test_validation():
    with pytest.raises(ValueError):
        LinkArbiter(0)
    arbiter, _ = make()
    with pytest.raises(ValueError):
        arbiter.register("wifi", 2)
