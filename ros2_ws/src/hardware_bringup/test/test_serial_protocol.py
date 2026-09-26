"""Tests for the Arduino line protocol helpers (no serial port needed)."""
from hardware_bringup.serial_protocol import (
    ENCODER_QUERY,
    LineSplitter,
    drive_command,
    is_error_response,
    parse_encoder_count,
)


def test_drive_command_is_integer_carriage_return_terminated():
    # firmware casts both args with (int) and terminates on '\r' (commands.h NEW_COMMAND)
    assert drive_command(-110, 45.7) == b'm -110 46\r'
    assert drive_command(0, 45.0) == b'm 0 45\r'
    assert drive_command(73.6, 20.2) == b'm 74 20\r'
    assert ENCODER_QUERY == b'e\r'


def test_line_splitter_handles_partial_lines_and_crlf():
    s = LineSplitter()
    assert s.feed(b'ddd Parsed - Vel: 0.00 | ddd Angle: 0.00\r\n123') == [
        b'ddd Parsed - Vel: 0.00 | ddd Angle: 0.00'
    ]
    assert s.feed(b'45\r\n') == [b'12345']
    assert s.feed(b'') == []
    assert s.feed(b'\r\n\r\n-7\n') == [b'-7']


def test_line_splitter_drops_runaway_buffer_without_newline():
    s = LineSplitter(max_buffer=16)
    assert s.feed(b'x' * 40) == []
    assert s.feed(b'99\r\n') == [b'99']


def test_parse_encoder_count_ignores_firmware_chatter():
    assert parse_encoder_count(b'12345') == 12345
    assert parse_encoder_count(b'-42') == -42
    assert parse_encoder_count(b'0') == 0
    assert parse_encoder_count(b'Parsed - Vel: 1 | Angle: 45') is None
    assert parse_encoder_count(b'ddd Parsed - Vel: 0.00 | ddd Angle: 0.00') is None
    assert parse_encoder_count(b'X') is None
    assert parse_encoder_count(b'') is None
    assert parse_encoder_count(b'12.5') is None
    assert parse_encoder_count(b'12.34') is None  # battery 'b' reply is a bare float


def test_error_response():
    assert is_error_response(b'X')
    assert not is_error_response(b'12')
