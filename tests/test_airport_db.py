"""Tests for engine.airport_db."""

import math
import pytest

from engine.airport_db import (
    load_airports,
    load_runway_ends,
    get_runway_end,
    wind_components,
)


def test_load_airports_returns_data():
    df = load_airports()
    assert len(df) > 1000
    assert "ident" in df.columns
    assert "display_name" in df.columns
    assert "elevation_ft" in df.columns


def test_load_airports_includes_ksez():
    df = load_airports()
    sez = df[df["ident"] == "KSEZ"]
    assert len(sez) == 1
    row = sez.iloc[0]
    # Sedona airport elevation is ~4830 ft
    assert 4700 < row["elevation_ft"] < 5000
    assert "SEDONA" in row["display_name"].upper()


def test_load_airports_excludes_heliports():
    df = load_airports()
    # Should not contain heliports
    assert not (df["type"].str.contains("HELI", case=False, na=False)).any()


def test_runway_ends_ksez_has_03_and_21():
    ends = load_runway_ends("KSEZ")
    idents = set(ends["rwy_ident"].tolist())
    # KSEZ has runway 03/21
    assert "03" in idents
    assert "21" in idents


def test_runway_ends_have_opposing_headings():
    ends = load_runway_ends("KSEZ")
    rwy03 = ends[ends["rwy_ident"] == "03"].iloc[0]
    rwy21 = ends[ends["rwy_ident"] == "21"].iloc[0]
    # Headings should be ~180° apart
    diff = abs(rwy03["heading_degT"] - rwy21["heading_degT"])
    assert 175 < diff < 185


def test_runway_ends_unknown_airport():
    ends = load_runway_ends("ZZZZ")
    assert ends.empty


def test_runway_ends_empty_input():
    ends = load_runway_ends("")
    assert ends.empty


def test_get_runway_end_lookup():
    end = get_runway_end("KSEZ", "03")
    assert end is not None
    assert end["rwy_ident"] == "03"
    assert end["length_ft"] > 4000  # KSEZ runway is ~5132 ft


def test_get_runway_end_missing():
    assert get_runway_end("KSEZ", "99") is None


def test_get_runway_end_case_insensitive():
    e1 = get_runway_end("ksez", "03")
    e2 = get_runway_end("KSEZ", "03")
    assert e1 is not None and e2 is not None
    assert e1["heading_degT"] == e2["heading_degT"]


# ── Wind components ──────────────────────────────────────────────────────────

def test_wind_components_pure_headwind():
    hw, xw, rel = wind_components(360.0, 10.0, 0.0)
    assert abs(hw - 10.0) < 0.01
    assert abs(xw) < 0.01


def test_wind_components_right_crosswind():
    # Wind from 090, runway 360 → 90° right crosswind
    hw, xw, rel = wind_components(90.0, 10.0, 0.0)
    assert abs(hw) < 0.01
    assert abs(xw - 10.0) < 0.01
    assert rel == 90.0


def test_wind_components_left_crosswind():
    # Wind from 270, runway 360 → 90° left crosswind (negative)
    hw, xw, rel = wind_components(270.0, 10.0, 0.0)
    assert abs(hw) < 0.01
    assert abs(xw + 10.0) < 0.01


def test_wind_components_tailwind():
    hw, xw, rel = wind_components(180.0, 10.0, 0.0)
    assert abs(hw + 10.0) < 0.01
    assert abs(xw) < 0.01


def test_wind_components_quartering():
    # 45° off the nose from the right at 10 kt
    hw, xw, rel = wind_components(45.0, 10.0, 0.0)
    expected = 10.0 * math.cos(math.radians(45))
    assert abs(hw - expected) < 0.01
    assert abs(xw - expected) < 0.01
