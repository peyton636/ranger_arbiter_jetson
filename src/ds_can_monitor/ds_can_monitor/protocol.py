"""STM32 CAN protocol: 0x110 main frame, 0x111 sensor distances."""

CAN_ID_MAIN = 0x110
CAN_ID_SENSOR = 0x111
CAN_ID_TEST = 0x7FF

MAX_VALID_DISTANCE_MM = 5000
INVALID_RAW_THRESHOLD = 5000  # e.g. 0xFFFD = 65533


def u16_be(data: bytes, index: int) -> int:
    return (data[index] << 8) | data[index + 1]


def parse_main_frame(data: bytes) -> dict:
    if len(data) < 8:
        raise ValueError("main frame needs 8 bytes")
    status = data[3]
    return {
        "version": data[0],
        "sensor_count": data[1],
        "beep_volume": data[2],
        "status": status,
        "data_valid": bool(status & 0x01),
        "can_ok": bool(status & 0x02),
        "sensor_status": [data[4], data[5], data[6], data[7]],
    }


def parse_distance(raw: int) -> int | None:
    if raw >= INVALID_RAW_THRESHOLD or raw == 0:
        return None
    return raw


def parse_sensor_frame(data: bytes) -> dict:
    if len(data) < 8:
        raise ValueError("sensor frame needs 8 bytes")
    distances = []
    valid = []
    for i in range(4):
        raw = u16_be(data, i * 2)
        dist = parse_distance(raw)
        distances.append(dist if dist is not None else raw)
        valid.append(dist is not None)
    valid_dists = [d for d, ok in zip(distances, valid) if ok]
    nearest = min(valid_dists) if valid_dists else None
    return {
        "distances": distances,
        "valid": valid,
        "nearest_mm": nearest,
    }
