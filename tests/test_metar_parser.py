"""Tests for engine.metar_parser."""

from engine.metar_parser import (
    parse_metar,
    isa_deviation_c,
    pressure_altitude_ft,
)


def test_parse_basic_metar():
    raw = "KSEZ 251853Z 24008KT 10SM CLR 22/M01 A3008"
    p = parse_metar(raw)
    assert p is not None
    assert p.station == "KSEZ"
    assert p.observation_time == "251853Z"
    assert p.wind_from_deg == 240
    assert p.wind_speed_kt == 8
    assert p.wind_gust_kt is None
    assert p.temperature_c == 22.0
    assert p.dewpoint_c == -1.0
    assert p.altimeter_inhg == 30.08


def test_parse_with_gust_and_variable_dir():
    raw = "KBOI 251953Z 31015G25KT 280V340 10SM SCT080 28/12 A2992 RMK AO2"
    p = parse_metar(raw)
    assert p.station == "KBOI"
    assert p.wind_from_deg == 310
    assert p.wind_speed_kt == 15
    assert p.wind_gust_kt == 25
    assert p.wind_var_from_deg == 280
    assert p.wind_var_to_deg == 340
    assert p.temperature_c == 28.0
    assert p.altimeter_inhg == 29.92


def test_parse_vrb_wind():
    raw = "KPAO 251953Z VRB03KT 10SM CLR 18/14 A3001"
    p = parse_metar(raw)
    assert p.wind_variable is True
    assert p.wind_from_deg is None
    assert p.wind_speed_kt == 3


def test_parse_negative_temps():
    raw = "PANC 251953Z 03012KT 10SM SCT040 M05/M12 A2985"
    p = parse_metar(raw)
    assert p.temperature_c == -5.0
    assert p.dewpoint_c == -12.0


def test_parse_q_altimeter_hpa():
    raw = "EGLL 251953Z 27010KT 9999 SCT030 15/08 Q1013"
    p = parse_metar(raw)
    assert p.altimeter_hpa == 1013.0
    assert p.altimeter_inhg == round(1013.0 * 0.02953, 2)


def test_parse_metar_prefix_and_remarks():
    raw = "METAR KJFK 251851Z 18012KT 10SM FEW250 24/15 A3010 RMK AO2 SLP193 T02390150"
    p = parse_metar(raw)
    assert p.station == "KJFK"
    # Make sure RMK group "T0239..." doesn't pollute temp parsing
    assert p.temperature_c == 24.0
    assert p.dewpoint_c == 15.0


def test_parse_empty_returns_none():
    assert parse_metar("") is None
    assert parse_metar("   ") is None


def test_isa_deviation_at_sea_level():
    # Standard day at sea level → 0 deviation
    assert abs(isa_deviation_c(15.0, 0.0)) < 0.01


def test_isa_deviation_hot_high():
    # KSEZ field elev ~4830 ft, ISA OAT ≈ 15 − 1.98×4.83 ≈ 5.4°C
    # If actual is 35°C, deviation ≈ 29.6°C
    dev = isa_deviation_c(35.0, 4830.0)
    assert 28.0 < dev < 31.0


def test_pressure_altitude_standard():
    assert pressure_altitude_ft(5000.0, 29.92) == 5000.0


def test_pressure_altitude_low_setting():
    # Low altimeter setting → higher pressure altitude
    pa = pressure_altitude_ft(5000.0, 29.42)
    assert pa == 5500.0
