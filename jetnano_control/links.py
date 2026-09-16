"""Transports that carry :class:`~jetnano_control.protocol.ControlFrame`.

Three links are provided:

* :class:`UdpLink`    -- plain datagrams over Wi-Fi, the primary link
* :class:`SerialLink` -- an HC-12 radio (or any UART) carrying frame lines
* :class:`ModbusLink` -- Modbus TCP holding registers, kept so Modbus
                         tooling from the original setup still works

The same class serves both ends of a link. The transmitter calls
:meth:`Link.start_sending` and drops frames into :meth:`Link.offer_to_send`;
a worker thread per link sends the newest frame at that link's own pace, so
a stalled link can never hold up the others. The receiver calls
:meth:`Link.start_receiving` and a worker thread per link hands every valid
frame to the sink. Worker threads never die on errors: they count them, try
to reopen the device, and back off.
"""
from __future__ import annotations

import abc
import logging
import socket
import threading
import time
from dataclasses import asdict, dataclass
from typing import Callable, List, Optional

from ._util import LogThrottle
from .protocol import WORD_COUNT, ControlFrame, FrameError

log = logging.getLogger("jetnano.links")

FrameSink = Callable[[str, ControlFrame, float], None]
StatusHandler = Callable[[str], None]
LineHandler = Callable[[bytes], None]

_STATUS_PREFIX = b"S"
_REOPEN_HOLDOFF_S = 1.0


class LinkError(RuntimeError):
    """A link could not do what it was asked."""


@dataclass(frozen=True)
class LinkSpec:
    """One link as written in the config file; both ends read the same spec."""
    type: str                   # "udp", "serial" or "modbus"
    name: str
    priority: int               # lower number wins on the receiver
    enabled: bool = True
    robot_host: str = ""        # udp/modbus: where the transmitter sends
    port: int = 0               # udp/modbus: port the receiver listens on
    bind_host: str = "0.0.0.0"  # udp/modbus: receiver bind address
    tx_port: str = ""           # serial: device on the transmitter
    rx_port: str = ""           # serial: device on the receiver
    baudrate: int = 9600
    unit_id: int = 1            # modbus
    base_address: int = 128     # modbus: first holding register of the frame
    timeout_s: float = 1.0      # modbus client timeout
    max_send_hz: float = 0.0    # 0 = send every frame


@dataclass
class LinkStats:
    sent: int = 0
    received: int = 0
    send_errors: int = 0
    receive_errors: int = 0
    decode_errors: int = 0
    reopens: int = 0
    last_error: str = ""
    last_sent_at: float = float("-inf")
    last_received_at: float = float("-inf")

    def as_dict(self) -> dict:
        return asdict(self)


class LatestMailbox:
    """A one-slot mailbox: putting a new frame replaces the old one."""

    def __init__(self) -> None:
        self._cv = threading.Condition()
        self._frame: Optional[ControlFrame] = None

    def put(self, frame: ControlFrame) -> None:
        with self._cv:
            self._frame = frame
            self._cv.notify()

    def take(self, timeout: float) -> Optional[ControlFrame]:
        with self._cv:
            if self._frame is None and timeout > 0:
                self._cv.wait(timeout)
            frame, self._frame = self._frame, None
            return frame

    def wake(self) -> None:
        """Release a waiting ``take`` so a stopping sender exits at once."""
        with self._cv:
            self._cv.notify_all()


class Link(abc.ABC):
    """Common behaviour: lifecycle, worker threads, error accounting."""

    def __init__(self, name: str, priority: int, max_send_hz: float = 0.0) -> None:
        self.name = name
        self.priority = priority
        self.stats = LinkStats()
        self.on_status: Optional[StatusHandler] = None
        self.on_aux_line: Optional[LineHandler] = None
        self._send_interval = 1.0 / max_send_hz if max_send_hz > 0 else 0.0
        self._mailbox = LatestMailbox()
        self._sink: Optional[FrameSink] = None
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []
        self._io_lock = threading.RLock()
        self._opened = False
        self._last_reopen = float("-inf")
        self._warn = LogThrottle(5.0)

    # ----- lifecycle -----------------------------------------------------
    def open(self) -> None:
        with self._io_lock:
            if self._opened:
                return
            self._open()
            self._opened = True
        log.info("%s: open", self.name)

    @property
    def is_open(self) -> bool:
        return self._opened

    def stop_workers(self) -> None:
        """Stop the send/receive threads but keep the device open."""
        self._stop.set()
        self._mailbox.wake()
        for thread in self._threads:
            thread.join(timeout=3.0)
        self._threads.clear()

    def close(self) -> None:
        self.stop_workers()
        with self._io_lock:
            if self._opened:
                try:
                    self._close()
                except Exception as exc:  # closing must never raise
                    log.debug("%s: error while closing: %s", self.name, exc)
                self._opened = False
        log.info("%s: closed", self.name)

    # ----- transmitting --------------------------------------------------
    def start_sending(self) -> None:
        self._spawn(self._send_loop, "send")

    def offer_to_send(self, frame: ControlFrame) -> None:
        self._mailbox.put(frame)

    def send_now(self, frame: ControlFrame) -> bool:
        """Send synchronously. Returns False (and counts the error) on failure."""
        try:
            with self._io_lock:
                self._send_raw(frame)
        except Exception as exc:
            self._note_error(exc, sending=True)
            return False
        self.stats.sent += 1
        self.stats.last_sent_at = time.monotonic()
        return True

    def _send_loop(self) -> None:
        next_allowed = 0.0
        backoff = 0.1
        while not self._stop.is_set():
            frame = self._mailbox.take(timeout=0.5)
            if frame is None:
                continue
            now = time.monotonic()
            if now < next_allowed:
                if self._stop.wait(next_allowed - now):
                    break
                newer = self._mailbox.take(timeout=0.0)   # prefer a newer frame
                if newer is not None:
                    frame = newer
            if self.send_now(frame):
                backoff = 0.1
                next_allowed = time.monotonic() + self._send_interval
            else:
                self._try_reopen()
                if self._stop.wait(backoff):
                    break
                backoff = min(backoff * 2, 5.0)

    # ----- receiving -----------------------------------------------------
    def start_receiving(self, sink: FrameSink) -> None:
        self._sink = sink
        self._spawn(self._receive_loop, "recv")

    def _receive_loop(self) -> None:
        backoff = 0.1
        while not self._stop.is_set():
            try:
                frame = self._receive_raw(0.5)
            except Exception as exc:
                if self._stop.is_set():
                    break
                self._note_error(exc, sending=False)
                self._try_reopen()
                if self._stop.wait(backoff):
                    break
                backoff = min(backoff * 2, 5.0)
                continue
            backoff = 0.1
            if frame is None:
                continue
            now = time.monotonic()
            self.stats.received += 1
            self.stats.last_received_at = now
            sink = self._sink
            if sink is not None:
                try:
                    sink(self.name, frame, now)
                except Exception:
                    log.exception("%s: frame sink failed", self.name)

    # ----- optional status back-channel ----------------------------------
    def send_status(self, text: str) -> bool:
        """Send a status line back to the transmitter, if this link can."""
        return False

    # ----- helpers -------------------------------------------------------
    def _spawn(self, target: Callable[[], None], kind: str) -> None:
        thread = threading.Thread(target=target, name=f"{self.name}-{kind}", daemon=True)
        thread.start()
        self._threads.append(thread)

    def _note_error(self, exc: Exception, sending: bool) -> None:
        if sending:
            self.stats.send_errors += 1
        else:
            self.stats.receive_errors += 1
        self.stats.last_error = f"{type(exc).__name__}: {exc}"
        kind = "send" if sending else "receive"
        if self._warn.allow(kind):
            log.warning("%s: %s error: %s", self.name, kind, self.stats.last_error)
        else:
            log.debug("%s: %s error: %s", self.name, kind, self.stats.last_error)

    def _note_decode_error(self, detail: str) -> None:
        self.stats.decode_errors += 1
        log.debug("%s: dropped frame: %s", self.name, detail)

    def _try_reopen(self) -> None:
        with self._io_lock:
            now = time.monotonic()
            if now - self._last_reopen < _REOPEN_HOLDOFF_S:
                return
            self._last_reopen = now
            try:
                self._close()
            except Exception:
                pass
            self._opened = False
            try:
                self._open()
                self._opened = True
                self.stats.reopens += 1
                log.info("%s: reopened", self.name)
            except Exception as exc:
                self.stats.last_error = f"{type(exc).__name__}: {exc}"
                log.debug("%s: reopen failed: %s", self.name, self.stats.last_error)

    # ----- subclass API --------------------------------------------------
    @abc.abstractmethod
    def _open(self) -> None: ...

    @abc.abstractmethod
    def _close(self) -> None: ...

    @abc.abstractmethod
    def _send_raw(self, frame: ControlFrame) -> None: ...

    @abc.abstractmethod
    def _receive_raw(self, timeout: float) -> Optional[ControlFrame]: ...


class UdpLink(Link):
    """Frames as UDP datagrams. Status lines from the receiver ride the same socket back."""

    def __init__(self, name: str, priority: int, *, local_port: int = 0, remote_host: str = "",
                 remote_port: int = 0, bind_host: str = "0.0.0.0", max_send_hz: float = 0.0) -> None:
        super().__init__(name, priority, max_send_hz)
        self.local_port = local_port
        self.bind_host = bind_host
        self.remote = (remote_host, remote_port) if remote_host else None
        self._sock: Optional[socket.socket] = None
        self._peer = None
        self._peer_lock = threading.Lock()

    def _open(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except OSError:
            pass
        sock.bind((self.bind_host, self.local_port))
        sock.settimeout(0.5)
        self._sock = sock

    def _close(self) -> None:
        sock, self._sock = self._sock, None
        if sock is not None:
            sock.close()

    @property
    def local_address(self):
        sock = self._sock
        return sock.getsockname() if sock is not None else None

    def _send_raw(self, frame: ControlFrame) -> None:
        if self.remote is None:
            raise LinkError("no remote host configured")
        sock = self._sock
        if sock is None:
            raise LinkError("socket closed")
        sock.sendto(frame.to_datagram(), self.remote)

    def _receive_raw(self, timeout: float) -> Optional[ControlFrame]:
        sock = self._sock
        if sock is None:
            raise LinkError("socket closed")
        sock.settimeout(timeout)
        try:
            data, addr = sock.recvfrom(512)
        except socket.timeout:
            return None
        if data[:1] == _STATUS_PREFIX:
            handler = self.on_status
            if handler is not None:
                handler(data[1:].decode("utf-8", errors="replace"))
            return None
        with self._peer_lock:
            self._peer = addr
        try:
            return ControlFrame.from_datagram(data)
        except FrameError as exc:
            self._note_decode_error(f"{exc} from {addr}")
            return None

    def send_status(self, text: str) -> bool:
        with self._peer_lock:
            peer = self._peer
        sock = self._sock
        if peer is None or sock is None:
            return False
        try:
            sock.sendto(_STATUS_PREFIX + text.encode("utf-8"), peer)
        except OSError as exc:
            log.debug("%s: status send failed: %s", self.name, exc)
            return False
        return True


class SerialLink(Link):
    """Frames as text lines on a serial port, such as an HC-12 radio.

    Lines that are not frames (anything not starting with ``$``) are handed
    to ``on_aux_line`` so other devices sharing the port, like a Teensy
    reporting battery voltage, can still be read.
    """

    def __init__(self, name: str, priority: int, *, port: str, baudrate: int = 9600,
                 max_send_hz: float = 15.0, read_timeout: float = 0.5,
                 serial_factory: Optional[Callable[[], object]] = None) -> None:
        super().__init__(name, priority, max_send_hz)
        self.port = port
        self.baudrate = baudrate
        self.read_timeout = read_timeout
        self._factory = serial_factory or self._default_factory
        self._ser = None

    def _default_factory(self):
        import serial  # pyserial, imported here so the module loads without it
        return serial.Serial(self.port, self.baudrate, timeout=self.read_timeout, write_timeout=1.0)

    def _open(self) -> None:
        ser = self._factory()
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        self._ser = ser

    def _close(self) -> None:
        ser, self._ser = self._ser, None
        if ser is not None:
            ser.close()

    def _send_raw(self, frame: ControlFrame) -> None:
        ser = self._ser
        if ser is None:
            raise LinkError("port closed")
        ser.write(frame.to_line())

    def _receive_raw(self, timeout: float) -> Optional[ControlFrame]:
        ser = self._ser
        if ser is None:
            raise LinkError("port closed")
        if ser.timeout != timeout:
            ser.timeout = timeout
        line = ser.readline(256)
        if not line:
            return None
        if not line.endswith(b"\n"):
            self._note_decode_error("partial line")
            return None
        if ControlFrame.looks_like_line(line):
            try:
                return ControlFrame.from_line(line)
            except FrameError as exc:
                self._note_decode_error(str(exc))
                return None
        handler = self.on_aux_line
        if handler is not None:
            handler(line)
        return None


def _make_waking_bank(base: int, count: int, changed: threading.Event):
    from pyModbusTCP.server import DataBank

    class WakingDataBank(DataBank):
        """A data bank that wakes the receiver when the frame registers change."""

        def on_holding_registers_change(self, address, from_value, to_value, srv_info):
            if base <= address < base + count:
                changed.set()

    return WakingDataBank()


class ModbusLink(Link):
    """Frames as nine Modbus holding registers.

    The receiver runs the Modbus server itself and is woken by its data bank
    when the registers change, so nothing polls over the network. The
    transmitter is a plain client writing the registers, which means any
    Modbus tool can also drive the robot for testing.
    """

    ROLE_CLIENT = "client"
    ROLE_SERVER = "server"

    def __init__(self, name: str, priority: int, *, role: str, host: str, port: int = 5020,
                 unit_id: int = 1, base_address: int = 128, timeout: float = 1.0,
                 max_send_hz: float = 20.0) -> None:
        super().__init__(name, priority, max_send_hz)
        if role not in (self.ROLE_CLIENT, self.ROLE_SERVER):
            raise ValueError(f"role must be 'client' or 'server', not {role!r}")
        self.role = role
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.base_address = base_address
        self.timeout = timeout
        self._client = None
        self._server = None
        self._bank = None
        self._changed = threading.Event()
        self._last_words: Optional[list] = None

    def _open(self) -> None:
        if self.role == self.ROLE_CLIENT:
            from pyModbusTCP.client import ModbusClient
            self._client = ModbusClient(host=self.host, port=self.port, unit_id=self.unit_id,
                                        timeout=self.timeout, auto_open=True)
            return
        from pyModbusTCP.server import ModbusServer
        self._bank = _make_waking_bank(self.base_address, WORD_COUNT, self._changed)
        server = ModbusServer(host=self.host, port=self.port, no_block=True, data_bank=self._bank)
        server.start()
        deadline = time.monotonic() + 2.0
        while not server.is_run and time.monotonic() < deadline:
            time.sleep(0.02)
        if not server.is_run:
            server.stop()
            raise LinkError(f"Modbus server failed to start on {self.host}:{self.port}")
        self._server = server

    def _close(self) -> None:
        client, self._client = self._client, None
        if client is not None:
            client.close()
        server, self._server = self._server, None
        if server is not None:
            server.stop()
        self._bank = None

    def _send_raw(self, frame: ControlFrame) -> None:
        client = self._client
        if client is None:
            raise LinkError("client not open")
        if not client.write_multiple_registers(self.base_address, frame.to_words()):
            raise LinkError(client.last_error_as_txt or "write failed")

    def _receive_raw(self, timeout: float) -> Optional[ControlFrame]:
        bank = self._bank
        if bank is None:
            raise LinkError("server not open")
        if not self._changed.wait(timeout):
            return None
        self._changed.clear()
        words = bank.get_holding_registers(self.base_address, WORD_COUNT)
        if not words or words == self._last_words:
            return None
        self._last_words = list(words)
        try:
            return ControlFrame.from_words(words)
        except FrameError as exc:
            self._note_decode_error(str(exc))
            return None


def build_link(spec: LinkSpec, role: str) -> Link:
    """Build the link a spec describes for the given end, ``"tx"`` or ``"rx"``."""
    if role not in ("tx", "rx"):
        raise ValueError(f"role must be 'tx' or 'rx', not {role!r}")
    if spec.type == "udp":
        if role == "tx":
            if not spec.robot_host:
                raise ValueError(f"link {spec.name!r}: robot_host is required")
            return UdpLink(spec.name, spec.priority, remote_host=spec.robot_host,
                           remote_port=spec.port, max_send_hz=spec.max_send_hz)
        return UdpLink(spec.name, spec.priority, local_port=spec.port, bind_host=spec.bind_host)
    if spec.type == "serial":
        port = spec.tx_port if role == "tx" else spec.rx_port
        if not port:
            raise ValueError(f"link {spec.name!r}: {'tx_port' if role == 'tx' else 'rx_port'} is required")
        return SerialLink(spec.name, spec.priority, port=port, baudrate=spec.baudrate,
                          max_send_hz=spec.max_send_hz)
    if spec.type == "modbus":
        if role == "tx":
            if not spec.robot_host:
                raise ValueError(f"link {spec.name!r}: robot_host is required")
            return ModbusLink(spec.name, spec.priority, role=ModbusLink.ROLE_CLIENT, host=spec.robot_host,
                              port=spec.port, unit_id=spec.unit_id, base_address=spec.base_address,
                              timeout=spec.timeout_s, max_send_hz=spec.max_send_hz)
        return ModbusLink(spec.name, spec.priority, role=ModbusLink.ROLE_SERVER, host=spec.bind_host,
                          port=spec.port, unit_id=spec.unit_id, base_address=spec.base_address)
    raise ValueError(f"link {spec.name!r}: unknown type {spec.type!r}")
