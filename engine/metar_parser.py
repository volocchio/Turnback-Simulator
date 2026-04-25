"""
METAR Parser
============

Lightweight regex-based parser for the fields the turnback simulator needs:
station, observation time, wind (dir/speed/gust, optional VRB), variable wind
range, temperature, dew point, altimeter (inHg or hPa).

This is NOT a full METAR parser — it ignores remarks, runway visual range,
weather phenomena, cloud layers, etc.  It is intentionally tolerant of
non-standard whitespace and mixed-case input so a pilot can paste straight
from ForeFlight / aviationweather.gov.

Public API
----------
    parse_metar(raw: str) -> ParsedMetar | None
"""

from __future__ import annotations

import re
from dataclasses import dataclass


@dataclass
class ParsedMetar:
    raw: str
    station: str = ""
    observation_time: str = ""        # e.g. "251853Z"
    wind_from_deg: int | None = None  # None if VRB
    wind_variable: bool = False
    wind_speed_kt: int = 0
    wind_gust_kt: int | None = None
    wind_var_from_deg: int | None = None  # variable-direction range "240V310"
    wind_var_to_deg: int | None = None
    temperature_c: float | None = None
    dewpoint_c: float | None = None
    altimeter_inhg: float | None = None  # populated from A#### group
    altimeter_hpa: float | None = None   # populated from Q#### group


_RE_STATION = re.compile(r"\b([A-Z][A-Z0-9]{3})\b")
_RE_TIME = re.compile(r"\b(\d{6}Z)\b")
_RE_WIND = re.compile(
    r"\b(VRB|\d{3})(\d{2,3})(?:G(\d{2,3}))?KT\b"
)
_RE_WIND_VAR = re.compile(r"\b(\d{3})V(\d{3})\b")
# Temperature / dew point group: e.g. "M03/M07" or "15/12" or "10/M01"
_RE_TEMP = re.compile(r"(?<![A-Z0-9])(M?\d{2})/(M?\d{2})(?![A-Z0-9])")
_RE_ALT_INHG = re.compile(r"\bA(\d{4})\b")
_RE_ALT_HPA = re.compile(r"\bQ(\d{3,4})\b")


def _to_int_with_minus(s: str) -> int:
    """Convert METAR temperature group ('M05' or '12') to signed int."""
    if s.startswith("M"):
        return -int(s[1:])
    return int(s)


def parse_metar(raw: str) -> ParsedMetar | None:
    """Parse a raw METAR string into a ParsedMetar. Returns None if input is empty.

    Tolerant of leading 'METAR ' / 'SPECI ' prefixes and trailing remarks
    after 'RMK'.  Unknown groups are silently ignored.
    """
    if not raw or not raw.strip():
        return None

    text = raw.strip().upper()
    # Strip METAR/SPECI prefix
    text = re.sub(r"^(METAR|SPECI)\s+", "", text)
    # Drop everything after RMK to avoid false matches in remarks
    text = re.split(r"\bRMK\b", text)[0]

    parsed = ParsedMetar(raw=raw.strip())

    # Station — first 4-char ICAO-looking token at the start
    tokens = text.split()
    if tokens and re.fullmatch(r"[A-Z][A-Z0-9]{3}", tokens[0]):
        parsed.station = tokens[0]

    m = _RE_TIME.search(text)
    if m:
        parsed.observation_time = m.group(1)

    m = _RE_WIND.search(text)
    if m:
        direction, speed, gust = m.group(1), m.group(2), m.group(3)
        if direction == "VRB":
            parsed.wind_variable = True
            parsed.wind_from_deg = None
        else:
            parsed.wind_from_deg = int(direction)
        parsed.wind_speed_kt = int(speed)
        if gust is not None:
            parsed.wind_gust_kt = int(gust)

    m = _RE_WIND_VAR.search(text)
    if m:
        parsed.wind_var_from_deg = int(m.group(1))
        parsed.wind_var_to_deg = int(m.group(2))

    m = _RE_TEMP.search(text)
    if m:
        try:
            parsed.temperature_c = float(_to_int_with_minus(m.group(1)))
            parsed.dewpoint_c = float(_to_int_with_minus(m.group(2)))
        except ValueError:
            pass

    m = _RE_ALT_INHG.search(text)
    if m:
        # "A2992" → 29.92 inHg
        parsed.altimeter_inhg = int(m.group(1)) / 100.0

    m = _RE_ALT_HPA.search(text)
    if m:
        # "Q1013" → 1013 hPa
        parsed.altimeter_hpa = float(m.group(1))

    # If only hPa given, derive inHg too (1 hPa = 0.02953 inHg)
    if parsed.altimeter_inhg is None and parsed.altimeter_hpa is not None:
        parsed.altimeter_inhg = round(parsed.altimeter_hpa * 0.02953, 2)
    # And vice versa
    if parsed.altimeter_hpa is None and parsed.altimeter_inhg is not None:
        parsed.altimeter_hpa = round(parsed.altimeter_inhg / 0.02953, 1)

    return parsed


# ── Derived quantities ────────────────────────────────────────────────────────

ISA_SEA_LEVEL_C = 15.0
ISA_LAPSE_C_PER_FT = 0.001981  # 1.98°C per 1000 ft
ISA_STD_ALTIMETER_INHG = 29.92


def isa_deviation_c(temperature_c: float, field_elev_ft: float) -> float:
    """Compute ISA deviation (°C) for a given OAT at a field elevation.

    ISA OAT = 15°C − 1.98°C/1000ft × elevation
    """
    isa_temp = ISA_SEA_LEVEL_C - ISA_LAPSE_C_PER_FT * field_elev_ft
    return temperature_c - isa_temp


def pressure_altitude_ft(field_elev_ft: float, altimeter_inhg: float) -> float:
    """Pressure altitude (ft) from field elevation and altimeter setting.

    PA = field_elev + (29.92 − altimeter) × 1000
    """
    return field_elev_ft + (ISA_STD_ALTIMETER_INHG - altimeter_inhg) * 1000.0
