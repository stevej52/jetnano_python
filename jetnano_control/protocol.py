"""The control frame every link carries.

One frame is one joystick sample: a sequence number, four axes, a button
mask and a flags word. The same nine 16-bit words are encoded three ways so
every transport carries identical data:

* ``words``    -- nine Modbus holding registers
* ``datagram`` -- the nine words packed big-endian, 18 bytes, for UDP
* ``line``     -- ``$`` + 36 upper-case hex digits + newline, for the radio

The last word is a CRC-16 over the first eight, so a frame damaged in
transit is rejected instead of steering the robot.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Sequence, Tuple

MAGIC = 0x4A43          # "JC"
WORD_COUNT = 9
DATAGRAM_SIZE = WORD_COUNT * 2
LINE_SIZE = 1 + DATAGRAM_SIZE * 2   # "$" plus hex digits, without the newline
AXIS_SCALE = 10000      # -1.0 .. 1.0 travels as -10000 .. 10000
AXIS_COUNT = 4

FLAG_ARMED = 0x0001     # operator is holding the arm (dead-man) button
FLAG_ESTOP = 0x0002     # operator asked for an emergency stop
FLAG_NEUTRAL = 0x0004   # transmitter has no usable joystick; hold neutral
FLAG_TURBO = 0x0008     # operator is holding the turbo button

_HEAD = struct.Struct(">8H")
_FRAME = struct.Struct(">9H")

Axes = Tuple[float, float, float, float]


class FrameError(ValueError):
    """Raised when bytes or words do not form a valid frame."""


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), a common serial-link CRC."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def clamp(value: float, low: float = -1.0, high: float = 1.0) -> float:
    return low if value < low else high if value > high else value


def seq_newer(candidate: int, reference: int) -> bool:
    """True when ``candidate`` comes after ``reference`` in 16-bit wrapping order."""
    return candidate != reference and ((candidate - reference) & 0xFFFF) < 0x8000


def _axis_to_word(value: float) -> int:
    return int(round(clamp(value) * AXIS_SCALE)) & 0xFFFF


def _word_to_axis(word: int) -> float:
    signed = word - 0x10000 if word > 0x7FFF else word
    return clamp(signed / AXIS_SCALE)


@dataclass(frozen=True)
class ControlFrame:
    seq: int
    axes: Axes
    buttons: int = 0
    flags: int = 0

    def __post_init__(self) -> None:
        axes = tuple(self.axes)
        if len(axes) != AXIS_COUNT:
            raise FrameError(f"expected {AXIS_COUNT} axes, got {len(axes)}")
        object.__setattr__(self, "seq", int(self.seq) & 0xFFFF)
        object.__setattr__(self, "axes", tuple(clamp(float(a)) for a in axes))
        object.__setattr__(self, "buttons", int(self.buttons) & 0xFFFF)
        object.__setattr__(self, "flags", int(self.flags) & 0xFFFF)

    # ----- flags ---------------------------------------------------------
    @property
    def armed(self) -> bool:
        return bool(self.flags & FLAG_ARMED)

    @property
    def estop(self) -> bool:
        return bool(self.flags & FLAG_ESTOP)

    @property
    def neutral(self) -> bool:
        return bool(self.flags & FLAG_NEUTRAL)

    @property
    def turbo(self) -> bool:
        return bool(self.flags & FLAG_TURBO)

    @classmethod
    def neutral_frame(cls, seq: int, flags: int = FLAG_NEUTRAL) -> "ControlFrame":
        return cls(seq=seq, axes=(0.0, 0.0, 0.0, 0.0), buttons=0, flags=flags)

    # ----- words (Modbus registers) --------------------------------------
    def to_words(self) -> list:
        head = [MAGIC, self.seq, *(_axis_to_word(a) for a in self.axes), self.buttons, self.flags]
        return head + [crc16(_HEAD.pack(*head))]

    @classmethod
    def from_words(cls, words: Sequence[int]) -> "ControlFrame":
        words = list(words)
        if len(words) != WORD_COUNT:
            raise FrameError(f"expected {WORD_COUNT} words, got {len(words)}")
        if any(not isinstance(w, int) or not 0 <= w <= 0xFFFF for w in words):
            raise FrameError("words must be integers in 0..65535")
        if words[0] != MAGIC:
            raise FrameError(f"bad magic 0x{words[0]:04X}")
        if crc16(_HEAD.pack(*words[:8])) != words[8]:
            raise FrameError("bad CRC")
        return cls(seq=words[1], axes=tuple(_word_to_axis(w) for w in words[2:6]),
                   buttons=words[6], flags=words[7])

    # ----- datagram (UDP) ------------------------------------------------
    def to_datagram(self) -> bytes:
        return _FRAME.pack(*self.to_words())

    @classmethod
    def from_datagram(cls, data: bytes) -> "ControlFrame":
        if len(data) != DATAGRAM_SIZE:
            raise FrameError(f"expected {DATAGRAM_SIZE} bytes, got {len(data)}")
        return cls.from_words(_FRAME.unpack(data))

    # ----- line (serial radio) -------------------------------------------
    def to_line(self) -> bytes:
        return b"$" + self.to_datagram().hex().upper().encode("ascii") + b"\n"

    @classmethod
    def from_line(cls, line: bytes) -> "ControlFrame":
        line = line.strip()
        if len(line) != LINE_SIZE or line[:1] != b"$":
            raise FrameError("not a frame line")
        try:
            data = bytes.fromhex(line[1:].decode("ascii"))
        except (ValueError, UnicodeDecodeError) as exc:
            raise FrameError("bad hex") from exc
        return cls.from_datagram(data)

    @staticmethod
    def looks_like_line(line: bytes) -> bool:
        return line.lstrip()[:1] == b"$"

    def describe(self) -> str:
        axes = " ".join(f"{a:+.2f}" for a in self.axes)
        return f"seq={self.seq} axes=[{axes}] buttons=0x{self.buttons:04X} flags=0x{self.flags:04X}"
