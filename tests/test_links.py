import queue
import socket
import threading
import time

import pytest
import serial

from conftest import wait_until
from jetnano_control.links import LatestMailbox, LinkSpec, ModbusLink, SerialLink, UdpLink, build_link
from jetnano_control.protocol import FLAG_ARMED, ControlFrame


def frame(seq):
    return ControlFrame(seq, (0.1, -0.2, 0.3, -0.4), buttons=3, flags=FLAG_ARMED)


class Collector:
    def __init__(self):
        self.items = queue.Queue()

    def __call__(self, name, frame, when):
        self.items.put((name, frame, when))

    def get(self, timeout=3.0):
        return self.items.get(timeout=timeout)

    def drain(self):
        out = []
        while True:
            try:
                out.append(self.items.get_nowait())
            except queue.Empty:
                return out


def free_port(kind=socket.SOCK_DGRAM):
    sock = socket.socket(socket.AF_INET, kind)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def test_latest_mailbox_keeps_only_newest():
    box = LatestMailbox()
    assert box.take(0.0) is None
    box.put(frame(1))
    box.put(frame(2))
    assert box.take(0.0).seq == 2
    assert box.take(0.0) is None


def test_udp_round_trip_status_backchannel_and_garbage():
    port = free_port()
    rx = UdpLink("wifi", 1, local_port=port, bind_host="127.0.0.1")
    tx = UdpLink("wifi", 1, remote_host="127.0.0.1", remote_port=port)
    got = Collector()
    statuses = queue.Queue()
    tx.on_status = statuses.put
    rx.open()
    tx.open()
    rx.start_receiving(got)
    tx.start_receiving(lambda *args: None)
    try:
        assert tx.send_now(frame(1))
        name, received, when = got.get()
        assert name == "wifi" and received == frame(1) and when <= time.monotonic()
        assert rx.send_status("active=wifi failsafe=0")
        assert statuses.get(timeout=3) == "active=wifi failsafe=0"
        raw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        raw.sendto(b"garbage", ("127.0.0.1", port))
        raw.close()
        assert wait_until(lambda: rx.stats.decode_errors == 1)
        assert tx.stats.sent == 1 and rx.stats.received == 1 and tx.stats.send_errors == 0
    finally:
        tx.close()
        rx.close()
    assert not tx.is_open and not rx.is_open


def test_udp_sender_thread_rate_limits_and_sends_newest():
    port = free_port()
    rx = UdpLink("wifi", 1, local_port=port, bind_host="127.0.0.1")
    tx = UdpLink("wifi", 1, remote_host="127.0.0.1", remote_port=port, max_send_hz=20)
    got = Collector()
    rx.open()
    tx.open()
    rx.start_receiving(got)
    tx.start_sending()
    try:
        for seq in range(100):
            tx.offer_to_send(frame(seq))
        time.sleep(0.35)
        assert 1 <= tx.stats.sent <= 10
        assert wait_until(lambda: rx.stats.received == tx.stats.sent)
        assert got.drain()[-1][1].seq == 99
    finally:
        tx.close()
        rx.close()


def test_udp_send_without_remote_fails_cleanly():
    link = UdpLink("wifi", 1)
    link.open()
    try:
        assert not link.send_now(frame(1))
        assert link.stats.send_errors == 1 and "no remote" in link.stats.last_error
        assert not link.send_status("x")
    finally:
        link.close()


def test_serial_round_trip_aux_lines_and_bad_frames():
    loop = serial.serial_for_url("loop://", timeout=0.1)
    tx = SerialLink("radio", 3, port="loop", serial_factory=lambda: loop)
    rx = SerialLink("radio", 3, port="loop", serial_factory=lambda: loop)
    got = Collector()
    aux = queue.Queue()
    rx.on_aux_line = aux.put
    rx.open()
    tx.open()
    rx.start_receiving(got)
    try:
        assert tx.send_now(frame(5))
        assert got.get()[1] == frame(5)
        loop.write(b"T:1234:5678\n")
        assert aux.get(timeout=3) == b"T:1234:5678\n"
        good = frame(6).to_line()
        loop.write(b"$" + b"Z" * (len(good) - 2) + b"\n")     # right length, not hex
        assert wait_until(lambda: rx.stats.decode_errors == 1)
        loop.write(good[:10])                                # partial line, then silence
        assert wait_until(lambda: rx.stats.decode_errors == 2)
        assert tx.send_now(frame(7))
        assert got.get()[1] == frame(7)
        assert rx.stats.received == 2
    finally:
        rx.close()
        tx.close()


class FlakySerial:
    """First instance dies on read; the second one works."""

    instances = []

    def __init__(self, fail):
        self.fail = fail
        self.lines = queue.Queue()
        self.timeout = 0.1
        self.closed = False
        FlakySerial.instances.append(self)

    def reset_input_buffer(self):
        pass

    def readline(self, size=-1):
        if self.fail:
            raise OSError("device unplugged")
        try:
            return self.lines.get(timeout=self.timeout)
        except queue.Empty:
            return b""

    def write(self, data):
        return len(data)

    def close(self):
        self.closed = True


def test_serial_link_reopens_after_device_failure():
    FlakySerial.instances = []
    calls = iter([True, False, False])
    rx = SerialLink("radio", 3, port="fake", serial_factory=lambda: FlakySerial(next(calls)))
    got = Collector()
    rx.open()
    rx.start_receiving(got)
    try:
        assert wait_until(lambda: rx.stats.reopens == 1)
        FlakySerial.instances[-1].lines.put(frame(8).to_line())
        assert got.get()[1] == frame(8)
        assert rx.stats.receive_errors == 1 and FlakySerial.instances[0].closed
    finally:
        rx.close()


def test_modbus_round_trip_and_rejects_bad_registers():
    port = free_port(socket.SOCK_STREAM)
    rx = ModbusLink("modbus", 2, role="server", host="127.0.0.1", port=port)
    tx = ModbusLink("modbus", 2, role="client", host="127.0.0.1", port=port, timeout=2.0)
    got = Collector()
    rx.open()
    tx.open()
    rx.start_receiving(got)
    try:
        assert tx.send_now(frame(9))
        assert got.get()[1] == frame(9)
        assert tx.send_now(frame(9))                       # unchanged registers: no new delivery
        with pytest.raises(queue.Empty):
            got.get(timeout=0.3)
        from pyModbusTCP.client import ModbusClient
        raw = ModbusClient(host="127.0.0.1", port=port, auto_open=True, timeout=2.0)
        words = frame(10).to_words()
        words[-1] ^= 1                                     # break the CRC
        assert raw.write_multiple_registers(128, words)
        assert wait_until(lambda: rx.stats.decode_errors == 1)
        raw.close()
        assert tx.send_now(frame(11))
        assert got.get()[1] == frame(11)
    finally:
        tx.close()
        rx.close()


def test_modbus_client_reports_unreachable_server():
    port = free_port(socket.SOCK_STREAM)
    tx = ModbusLink("modbus", 2, role="client", host="127.0.0.1", port=port, timeout=0.5)
    tx.open()
    try:
        assert not tx.send_now(frame(1))
        assert tx.stats.send_errors == 1
    finally:
        tx.close()


def test_build_link_from_specs():
    udp = LinkSpec(type="udp", name="wifi", priority=1, robot_host="10.0.0.1", port=5555, max_send_hz=50)
    tx = build_link(udp, "tx")
    assert isinstance(tx, UdpLink) and tx.remote == ("10.0.0.1", 5555) and tx.local_port == 0
    rx = build_link(udp, "rx")
    assert isinstance(rx, UdpLink) and rx.remote is None and rx.local_port == 5555
    ser = LinkSpec(type="serial", name="radio", priority=3, tx_port="/dev/a", rx_port="/dev/b", baudrate=19200)
    assert build_link(ser, "tx").port == "/dev/a" and build_link(ser, "rx").port == "/dev/b"
    assert build_link(ser, "rx").baudrate == 19200
    mb = LinkSpec(type="modbus", name="modbus", priority=2, robot_host="10.0.0.1", port=5020)
    assert build_link(mb, "tx").role == "client" and build_link(mb, "rx").role == "server"
    with pytest.raises(ValueError):
        build_link(LinkSpec(type="udp", name="x", priority=1, port=1), "tx")
    with pytest.raises(ValueError):
        build_link(LinkSpec(type="serial", name="x", priority=1), "rx")
    with pytest.raises(ValueError):
        build_link(LinkSpec(type="pigeon", name="x", priority=1), "rx")
    with pytest.raises(ValueError):
        build_link(udp, "sideways")
