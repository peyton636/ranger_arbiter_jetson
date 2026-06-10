"""STM32 USART3 binary frame: 12 bytes, header 0xFF, XOR checksum."""

FRAME_HEADER = 0xFF
FRAME_LEN = 12
SENSOR_COUNT_EXPECTED = 4

MIN_VALID_DISTANCE_MM = 10
MAX_VALID_DISTANCE_MM = 3000
INVALID_DISTANCE_RAW = 0xFFFF


def u16_be(data: bytes, index: int) -> int:
    return (data[index] << 8) | data[index + 1]


def xor_checksum(data: bytes) -> int:
    checksum = 0
    for b in data:
        checksum ^= b
    return checksum


def is_valid_distance(raw: int) -> bool:
    if raw == INVALID_DISTANCE_RAW or raw == 0:
        return False
    if raw < MIN_VALID_DISTANCE_MM or raw > MAX_VALID_DISTANCE_MM:
        return False
    return True


def parse_frame(frame: bytes) -> dict | None:
    """Parse and validate one 12-byte frame. Returns None if invalid."""
    if len(frame) != FRAME_LEN:
        return None
    if frame[0] != FRAME_HEADER:
        return None

    body = frame[:-1]
    if xor_checksum(body) != frame[11]:
        return None

    distances_raw = [u16_be(frame, 2 + i * 2) for i in range(4)]
    valid = [is_valid_distance(d) for d in distances_raw]
    distances = [
        d if ok else None for d, ok in zip(distances_raw, valid)
    ]

    valid_mm = [d for d in distances if d is not None]
    nearest = min(valid_mm) if valid_mm else None

    return {
        "sensor_count": frame[1],
        "distances_raw": distances_raw,
        "distances": distances,
        "valid": valid,
        "beep_duty": frame[10],
        "checksum_ok": True,
        "nearest_mm": nearest,
    }


class FrameParser:
    """Buffer stream data and extract 12-byte frames aligned on 0xFF."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[dict]:
        self._buf.extend(data)
        results: list[dict] = []

        while len(self._buf) >= FRAME_LEN:
            if self._buf[0] != FRAME_HEADER:
                self._buf.pop(0)
                continue

            frame = bytes(self._buf[:FRAME_LEN])
            parsed = parse_frame(frame)
            del self._buf[:FRAME_LEN]

            if parsed is not None:
                results.append(parsed)

        return results
