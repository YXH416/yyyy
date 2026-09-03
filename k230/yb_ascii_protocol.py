"""Reusable Yahboom K230 ASCII protocol encoder and decoder.

Wire format:
    $<total_length>,<function_id>,<value0>,...,#

The total length includes both '$' and '#'. This module has no camera or UART
dependency, so the same encoder can be reused by color, rectangle, or target
coordinate programs.
"""

FRAME_HEAD = "$"
FRAME_TAIL = "#"
FIELD_SEPARATOR = ","
FUNCTION_COLOR_RECTANGLE = 1
FUNCTION_TARGET_RECTANGLE = 15
FUNCTION_TARGET_CENTER = 16
MAX_FRAME_LENGTH = 64
RELATIVE_X_MIN = -32768
RELATIVE_X_MAX = 32767


def _integer(value, name):
    try:
        converted = int(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be an integer" % name)
    return converted


def encode_frame(function_id, *values):
    """Return one self-consistent ASCII frame as a string."""
    function_id = _integer(function_id, "function_id")
    if function_id < 0 or function_id > 255:
        raise ValueError("function_id must be in range 0..255")

    fields = [str(function_id)]
    for index, value in enumerate(values):
        fields.append(str(_integer(value, "value%d" % index)))
    body = FIELD_SEPARATOR.join(fields) + FIELD_SEPARATOR + FRAME_TAIL

    # The number of digits in total_length is itself part of total_length.
    declared_length = 0
    for _ in range(4):
        packet = FRAME_HEAD + str(declared_length) + FIELD_SEPARATOR + body
        actual_length = len(packet)
        if actual_length == declared_length:
            if actual_length > MAX_FRAME_LENGTH:
                raise ValueError("encoded frame exceeds MAX_FRAME_LENGTH")
            return packet
        declared_length = actual_length
    raise ValueError("could not stabilize encoded frame length")


def decode_frame(packet):
    """Strictly decode a frame and return (function_id, payload_tuple)."""
    if isinstance(packet, bytes):
        packet = packet.decode("ascii")
    if not isinstance(packet, str):
        raise ValueError("packet must be str or bytes")
    if (len(packet) < 7 or len(packet) > MAX_FRAME_LENGTH or
            packet[0] != FRAME_HEAD or packet[-1] != FRAME_TAIL or
            packet[-2] != FIELD_SEPARATOR):
        raise ValueError("invalid frame delimiters or length")

    text_fields = packet[1:-2].split(FIELD_SEPARATOR)
    if len(text_fields) < 2 or any(field == "" for field in text_fields):
        raise ValueError("invalid field count")
    try:
        fields = tuple(int(field) for field in text_fields)
    except ValueError:
        raise ValueError("non-integer field")
    if fields[0] != len(packet):
        raise ValueError("declared length does not match packet")
    if fields[1] < 0 or fields[1] > 255:
        raise ValueError("function_id must be in range 0..255")
    return fields[1], fields[2:]


def encode_rectangle(function_id, x, y, width, height):
    """Encode a top-left rectangle using the verified x/y/w/h field order."""
    x = _integer(x, "x")
    y = _integer(y, "y")
    width = _integer(width, "width")
    height = _integer(height, "height")
    if x < 0 or y < 0 or width <= 0 or height <= 0:
        raise ValueError("rectangle requires x/y >= 0 and width/height > 0")
    return encode_frame(function_id, x, y, width, height)


def rectangle_center(x, y, width, height):
    """Convert a top-left rectangle to the integer center used by MSPM0."""
    x = _integer(x, "x")
    y = _integer(y, "y")
    width = _integer(width, "width")
    height = _integer(height, "height")
    if x < 0 or y < 0 or width <= 0 or height <= 0:
        raise ValueError("rectangle requires x/y >= 0 and width/height > 0")
    return x + width // 2, y + height // 2


def encode_center(center_x, center_y):
    """Encode an exact target center using function ID 16."""
    center_x = _integer(center_x, "center_x")
    center_y = _integer(center_y, "center_y")
    if center_x < 0 or center_y < 0:
        raise ValueError("center coordinates must be >= 0")
    return encode_frame(FUNCTION_TARGET_CENTER, center_x, center_y)


def encode_relative_x(relative_x, frame_dt_ms=0):
    """Encode signed pipe-centred X and its measurement interval.

    ``frame_dt_ms`` is the time since the previous transmitted position.
    Zero retains compatibility with receivers using the older reserved field.
    """
    relative_x = _integer(relative_x, "relative_x")
    if relative_x < RELATIVE_X_MIN or relative_x > RELATIVE_X_MAX:
        raise ValueError("relative_x must fit a signed 16-bit integer")
    frame_dt_ms = _integer(frame_dt_ms, "frame_dt_ms")
    if frame_dt_ms < 0 or frame_dt_ms > 60000:
        raise ValueError("frame_dt_ms must be in range 0..60000")
    # Keep the previous two-value payload shape and reuse its reserved field.
    return encode_frame(
        FUNCTION_TARGET_CENTER, relative_x, frame_dt_ms
    )


def send_packet(uart, packet):
    """Send through Yahboom YbUart.send() or machine.UART.write()."""
    if hasattr(uart, "send"):
        return uart.send(packet)
    if hasattr(uart, "write"):
        try:
            return uart.write(packet)
        except TypeError:
            return uart.write(packet.encode("ascii"))
    raise ValueError("UART object must provide send() or write()")


def send_rectangle(uart, function_id, x, y, width, height):
    """Encode and send one rectangle; return the exact transmitted packet."""
    packet = encode_rectangle(function_id, x, y, width, height)
    send_packet(uart, packet)
    return packet


def send_center(uart, center_x, center_y):
    """Encode and send one exact target center; return the packet."""
    packet = encode_center(center_x, center_y)
    send_packet(uart, packet)
    return packet


def send_relative_x(uart, relative_x, frame_dt_ms=0):
    """Send ``$len,16,relative_x,frame_dt_ms,#``."""
    packet = encode_relative_x(relative_x, frame_dt_ms)
    send_packet(uart, packet)
    return packet


