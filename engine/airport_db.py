"""
Airport & Runway Database
=========================

Loads OurAirports CSV data (airports_full.csv, runways.csv) into pandas
DataFrames suitable for Streamlit selectboxes and runway-end lookups.

Public API
----------
    load_airports()                  -> DataFrame (one row per airport)
    load_runway_ends(ident)          -> DataFrame (one row per runway END)
    get_runway_end(ident, rwy_ident) -> dict | None
"""

from __future__ import annotations

import os
from pathlib import Path

import pandas as pd

_DATA_DIR = Path(__file__).resolve().parent.parent / "data"
_AIRPORTS_CSV = _DATA_DIR / "airports_full.csv"
_RUNWAYS_CSV = _DATA_DIR / "runways.csv"


# ── Optional Streamlit caching ────────────────────────────────────────────────
try:
    import streamlit as st
    _cache = st.cache_data
except Exception:  # pragma: no cover — non-Streamlit context (tests)
    def _cache(func):
        return func


# ── Airports ──────────────────────────────────────────────────────────────────

@_cache
def load_airports() -> pd.DataFrame:
    """Load airports_full.csv with normalized text columns and a display_name.

    Returns a DataFrame sorted by display_name with columns including:
        ident, type, name, latitude_deg, longitude_deg, elevation_ft,
        municipality, iso_country, display_name
    """
    if not _AIRPORTS_CSV.exists():
        raise FileNotFoundError(f"Missing airport database: {_AIRPORTS_CSV}")

    df = pd.read_csv(_AIRPORTS_CSV)
    df = df.dropna(subset=["latitude_deg", "longitude_deg"])
    df["elevation_ft"] = df["elevation_ft"].fillna(0)

    text_cols = [
        "ident", "type", "name", "municipality", "iso_country",
        "iso_region", "gps_code", "iata_code", "local_code",
    ]
    for col in text_cols:
        if col in df.columns:
            df[col] = df[col].astype(str).str.upper()

    # Filter out heliports/seaplane bases/closed — turnback only relevant for
    # fixed-wing departures from a real runway airport.
    suitable = {"SMALL_AIRPORT", "MEDIUM_AIRPORT", "LARGE_AIRPORT"}
    df["type_upper"] = df["type"].str.upper().str.replace(" ", "_")
    df = df[df["type_upper"].isin(suitable)].copy()
    df = df.drop(columns=["type_upper"])

    ident = df["ident"].astype(str)
    name = df["name"].astype(str)
    muni = df["municipality"].fillna("").astype(str)
    has_muni = muni.str.len().gt(0) & muni.str.upper().ne("NAN")

    df["display_name"] = ident + " — " + name
    df.loc[has_muni, "display_name"] = (
        df.loc[has_muni, "display_name"] + " (" + muni.loc[has_muni] + ")"
    )

    df = df.sort_values(by="display_name").reset_index(drop=True)
    return df


# ── Runways ───────────────────────────────────────────────────────────────────

@_cache
def _load_runways_raw() -> pd.DataFrame:
    """Load runways.csv as-is (drops closed runways)."""
    if not _RUNWAYS_CSV.exists():
        raise FileNotFoundError(f"Missing runway database: {_RUNWAYS_CSV}")
    cols = [
        "airport_ident", "length_ft", "width_ft", "surface", "lighted", "closed",
        "le_ident", "le_latitude_deg", "le_longitude_deg",
        "le_elevation_ft", "le_heading_degT", "le_displaced_threshold_ft",
        "he_ident", "he_latitude_deg", "he_longitude_deg",
        "he_elevation_ft", "he_heading_degT", "he_displaced_threshold_ft",
    ]
    rwy = pd.read_csv(_RUNWAYS_CSV, usecols=lambda c: c in cols)
    rwy = rwy[rwy["closed"] != 1].copy()
    rwy["airport_ident"] = rwy["airport_ident"].astype(str).str.upper()
    rwy["length_ft"] = pd.to_numeric(rwy["length_ft"], errors="coerce")
    rwy = rwy.dropna(subset=["length_ft"])
    return rwy


def load_runway_ends(airport_ident: str) -> pd.DataFrame:
    """Return one row per runway END for the given airport.

    Each runway is split into its low-end (le_*) and high-end (he_*) rows so
    the user can pick a specific runway direction (e.g. "03" vs "21" at KSEZ).

    Columns: rwy_ident, length_ft, width_ft, surface, heading_degT,
             threshold_elevation_ft, displaced_threshold_ft, lat, lon
    """
    if not airport_ident:
        return pd.DataFrame()

    ident = str(airport_ident).upper()
    rwy = _load_runways_raw()
    rwy = rwy[rwy["airport_ident"] == ident]
    if rwy.empty:
        return pd.DataFrame()

    common_cols = ["length_ft", "width_ft", "surface"]

    le = rwy[common_cols + [
        "le_ident", "le_heading_degT", "le_elevation_ft",
        "le_displaced_threshold_ft", "le_latitude_deg", "le_longitude_deg",
    ]].rename(columns={
        "le_ident": "rwy_ident",
        "le_heading_degT": "heading_degT",
        "le_elevation_ft": "threshold_elevation_ft",
        "le_displaced_threshold_ft": "displaced_threshold_ft",
        "le_latitude_deg": "lat",
        "le_longitude_deg": "lon",
    })

    he = rwy[common_cols + [
        "he_ident", "he_heading_degT", "he_elevation_ft",
        "he_displaced_threshold_ft", "he_latitude_deg", "he_longitude_deg",
    ]].rename(columns={
        "he_ident": "rwy_ident",
        "he_heading_degT": "heading_degT",
        "he_elevation_ft": "threshold_elevation_ft",
        "he_displaced_threshold_ft": "displaced_threshold_ft",
        "he_latitude_deg": "lat",
        "he_longitude_deg": "lon",
    })

    ends = pd.concat([le, he], ignore_index=True)
    ends = ends.dropna(subset=["rwy_ident"])
    ends["rwy_ident"] = ends["rwy_ident"].astype(str).str.upper().str.strip()
    ends = ends[ends["rwy_ident"] != ""]
    ends = ends[~ends["rwy_ident"].isin(["NAN", "NONE"])]

    # Sort by numeric runway number when possible (03 before 21, etc.)
    def _rwy_sort_key(s: str) -> tuple:
        digits = "".join(c for c in s if c.isdigit())
        try:
            return (int(digits) if digits else 99, s)
        except ValueError:
            return (99, s)

    ends = ends.assign(_sort=ends["rwy_ident"].map(_rwy_sort_key))
    ends = ends.sort_values(by="_sort").drop(columns=["_sort"]).reset_index(drop=True)
    return ends


def get_runway_end(airport_ident: str, rwy_ident: str) -> dict | None:
    """Look up a single runway end. Returns dict or None."""
    ends = load_runway_ends(airport_ident)
    if ends.empty:
        return None
    target = str(rwy_ident).upper().strip()
    match = ends[ends["rwy_ident"] == target]
    if match.empty:
        return None
    return match.iloc[0].to_dict()


# ── Wind component helper ─────────────────────────────────────────────────────

def wind_components(wind_from_deg_true: float, wind_speed_kt: float,
                    runway_heading_deg_true: float) -> tuple[float, float, float]:
    """Compute headwind, crosswind, and relative wind angle.

    Args:
        wind_from_deg_true: METAR/forecast wind FROM direction (deg true).
        wind_speed_kt: wind speed in knots.
        runway_heading_deg_true: runway magnetic-true heading.

    Returns:
        (headwind_kt, crosswind_kt, relative_angle_deg)
        - headwind_kt > 0 = headwind, < 0 = tailwind
        - crosswind_kt > 0 = right cross, < 0 = left cross
        - relative_angle_deg = wind_from − runway_heading, normalized to [0, 360)
    """
    import math
    rel = (float(wind_from_deg_true) - float(runway_heading_deg_true)) % 360.0
    rel_rad = math.radians(rel)
    headwind = wind_speed_kt * math.cos(rel_rad)
    crosswind = wind_speed_kt * math.sin(rel_rad)
    return headwind, crosswind, rel
