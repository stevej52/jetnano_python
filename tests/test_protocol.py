import pytest

from jetnano_control.protocol import (DATAGRAM_SIZE, FLAG_ARMED, FLAG_ESTOP, FLAG_NEUTRAL, LINE_SIZE,
                                      WORD_COUNT, ControlFrame, FrameError, crc16, seq_newer)


def test_round_trip_words_datagram_line():
    frame = ControlFrame(seq=1234, axes=(-1.0, -0.5, 0.25, 1.0), buttons=0xBEEF, flags=FLAG_ARMED | FLAG_ESTOP)
    words = frame.to_words()
    datagram = frame.to_datagram()
    line = frame.to_line()
    assert len(words) == WORD_COUNT
    assert len(datagram) == DATAGRAM_SIZE
    assert len(line) == LINE_SIZE + 1 and line.startswith(b"$") and line.endswith(b"\n")
    assert ControlFrame.from_words(words) == frame
    assert ControlFrame.from_datagram(datagram) == frame
    assert ControlFrame.from_line(line) == frame
    assert ControlFrame.from_line(b"  " + line + b"\r\n") == frame


def test_axes_are_clamped_and_quantised():
    frame = ControlFrame(seq=0, axes=(2.0, -3.0, 0.123456, 0.0))
    assert frame.axes[0] == 1.0 and frame.axes[1] == -1.0
    decoded = ControlFrame.from_words(frame.to_words())
    assert decoded.axes[2] == pytest.approx(0.1235)
    assert decoded.axes[0] == 1.0 and decoded.axes[1] == -1.0


def test_seq_buttons_flags_wrap_to_16_bits():
    frame = ControlFrame(seq=0x1FFFF, axes=(0, 0, 0, 0), buttons=0x12345, flags=0x10003)
    assert frame.seq == 0xFFFF and frame.buttons == 0x2345 and frame.flags == 3


def test_corruption_is_rejected():
    frame = ControlFrame(seq=7, axes=(0.1, 0.2, 0.3, 0.4), buttons=1, flags=FLAG_ARMED)
    words = frame.to_words()
    flipped = list(words)
    flipped[3] ^= 0x0100
    with pytest.raises(FrameError, match="CRC"):
        ControlFrame.from_words(flipped)
    wrong_magic = list(words)
    wrong_magic[0] = 0x1234
    with pytest.raises(FrameError, match="magic"):
        ControlFrame.from_words(wrong_magic)
    with pytest.raises(FrameError):
        ControlFrame.from_words(words[:-1])
    with pytest.raises(FrameError):
        ControlFrame.from_words(words[:-1] + [70000])
    data = bytearray(frame.to_datagram())
    data[5] ^= 0xFF
    with pytest.raises(FrameError):
        ControlFrame.from_datagram(bytes(data))
    with pytest.raises(FrameError):
        ControlFrame.from_datagram(frame.to_datagram()[:-1])
    line = frame.to_line()
    with pytest.raises(FrameError):
        ControlFrame.from_line(line[:5] + b"ZZ" + line[7:])
    with pytest.raises(FrameError):
        ControlFrame.from_line(b"hello\n")
    with pytest.raises(FrameError):
        ControlFrame.from_line(line[:-3] + b"\n")


def test_wrong_axis_count():
    with pytest.raises(FrameError):
        ControlFrame(seq=0, axes=(0, 0, 0))


def test_flags_and_neutral_frame():
    neutral = ControlFrame.neutral_frame(5)
    assert neutral.neutral and not neutral.armed and not neutral.estop and neutral.axes == (0, 0, 0, 0)
    armed = ControlFrame(1, (0, 0, 0, 0), flags=FLAG_ARMED)
    assert armed.armed and not armed.estop and not armed.neutral and not armed.turbo
    assert ControlFrame.neutral_frame(2, FLAG_NEUTRAL | FLAG_ESTOP).estop


def test_seq_newer_wraps():
    assert seq_newer(1, 0)
    assert not seq_newer(0, 1)
    assert not seq_newer(5, 5)
    assert seq_newer(0, 0xFFFF)
    assert not seq_newer(0xFFFF, 0)
    assert seq_newer(0x7FFF, 0)
    assert not seq_newer(0x8000, 0)


def test_crc16_known_check_value():
    assert crc16(b"123456789") == 0x29B1


def test_looks_like_line_and_describe():
    frame = ControlFrame(3, (0, 0, 0, 0))
    assert ControlFrame.looks_like_line(frame.to_line())
    assert not ControlFrame.looks_like_line(b"T:1234\n")
    assert "seq=3" in frame.describe()
