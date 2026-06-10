# BSD-licensed NMEA parser (adapted from nmea_navsat_driver / Eric Perko)
import calendar
import math
import re
import time


def check_nmea_checksum(nmea_sentence):
    if isinstance(nmea_sentence, bytes):
        nmea_sentence = nmea_sentence.decode("ascii", errors="ignore")
    split_sentence = nmea_sentence.split("*")
    if len(split_sentence) != 2:
        return False
    transmitted_checksum = split_sentence[1].strip()
    checksum = 0
    for c in split_sentence[0][1:]:
        checksum ^= ord(c)
    return ("%02X" % checksum) == transmitted_checksum.upper()


def safe_float(field):
    try:
        return float(field)
    except ValueError:
        return float("nan")


def safe_int(field):
    try:
        return int(field)
    except ValueError:
        return 0


def convert_latitude(field):
    return safe_float(field[0:2]) + safe_float(field[2:]) / 60.0


def convert_longitude(field):
    return safe_float(field[0:3]) + safe_float(field[3:]) / 60.0


def convert_time(nmea_utc):
    utc_list = list(time.gmtime())
    if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
        return float("nan")
    utc_list[3] = int(nmea_utc[0:2])
    utc_list[4] = int(nmea_utc[2:4])
    utc_list[5] = int(nmea_utc[4:6])
    return calendar.timegm(tuple(utc_list))


def convert_status_flag(status_flag):
    return status_flag == "A"


def convert_knots_to_mps(knots):
    return safe_float(knots) * 0.514444444444


def convert_deg_to_rads(degs):
    return math.radians(safe_float(degs))


PARSE_MAPS = {
    "GGA": [
        ("fix_type", int, 6),
        ("latitude", convert_latitude, 2),
        ("latitude_direction", str, 3),
        ("longitude", convert_longitude, 4),
        ("longitude_direction", str, 5),
        ("altitude", safe_float, 9),
        ("mean_sea_level", safe_float, 11),
        ("hdop", safe_float, 8),
        ("num_satellites", safe_int, 7),
        ("utc_time", convert_time, 1),
    ],
    "RMC": [
        ("utc_time", convert_time, 1),
        ("fix_valid", convert_status_flag, 2),
        ("latitude", convert_latitude, 3),
        ("latitude_direction", str, 4),
        ("longitude", convert_longitude, 5),
        ("longitude_direction", str, 6),
        ("speed", convert_knots_to_mps, 7),
        ("true_course", convert_deg_to_rads, 8),
    ],
    "VTG": [
        ("true_course", safe_float, 1),
        ("speed", convert_knots_to_mps, 5),
    ],
}


def parse_nmea_sentence(nmea_sentence):
    if isinstance(nmea_sentence, bytes):
        nmea_sentence = nmea_sentence.decode("ascii", errors="ignore")
    nmea_sentence = nmea_sentence.strip()
    if not re.match(
        r"(^\$GP|^\$GN|^\$GL|^\$IN|^\$BD).*\*[0-9A-Fa-f]{2}$", nmea_sentence
    ):
        return None

    fields = [field.strip(",") for field in nmea_sentence.split(",")]
    fields[-1] = fields[-1].split("*")[0]
    sentence_type = fields[0][3:]
    if sentence_type not in PARSE_MAPS:
        return None

    parsed = {}
    for name, converter, index in PARSE_MAPS[sentence_type]:
        if index >= len(fields) or fields[index] == "":
            if name in ("fix_type", "num_satellites"):
                parsed[name] = 0
            elif name.endswith("_direction") or name == "fix_valid":
                parsed[name] = "" if name.endswith("_direction") else False
            else:
                parsed[name] = float("nan")
        else:
            parsed[name] = converter(fields[index])
    return {sentence_type: parsed}
