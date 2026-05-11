"""
Forced-Landing Satellite Map
============================

Renders an Esri World Imagery satellite map of a user-chosen airport and
overlays the glide footprint at the dead-zone altitudes (the band where
the aircraft is too high to land straight ahead on the remaining runway
and too low to complete the turnback). Optionally queries OpenStreetMap
(Overpass API) for nearby landing candidates and hazards within glide
range.

Coordinate conventions
----------------------
- All headings are degrees true, 0 = North, increasing clockwise.
- Glide footprint is computed from the engine-failure point (which is
  estimated by projecting forward along the runway centerline using a
  typical climb gradient).
- L/D used is the aerodynamic best-glide L/D from the simulator
  (already accounts for prop drag, gear, flaps).

This module is UI-agnostic; the only Streamlit dependency is in
``render_landing_map_section``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

# Earth radius in meters (mean)
_EARTH_R_M = 6_371_000.0
_FT_PER_M = 3.28084
_M_PER_NM = 1852.0


# ──────────────────────────────────────────────────────────────────────
# Airport lookup
# ──────────────────────────────────────────────────────────────────────

@dataclass
class Airport:
    code: str          # ICAO if known, else IATA
    icao: str
    iata: str
    name: str
    city: str
    country: str
    lat: float
    lon: float
    elevation_ft: float


def lookup_airport(code: str) -> Optional[Airport]:
    """Look up an airport by ICAO (4-letter) or IATA (3-letter) code.

    Uses the offline ``airportsdata`` package (no network). Returns None
    if not found.
    """
    if not code:
        return None
    code = code.strip().upper()
    try:
        import airportsdata
    except ImportError:
        return None

    # Try ICAO first (4-letter), then IATA (3-letter)
    if len(code) == 4:
        db = airportsdata.load('ICAO')
        rec = db.get(code)
    elif len(code) == 3:
        db = airportsdata.load('IATA')
        rec = db.get(code)
    else:
        # Try both
        rec = airportsdata.load('ICAO').get(code) or \
              airportsdata.load('IATA').get(code)

    if not rec:
        return None
    return Airport(
        code=code,
        icao=rec.get('icao', ''),
        iata=rec.get('iata', ''),
        name=rec.get('name', ''),
        city=rec.get('city', ''),
        country=rec.get('country', ''),
        lat=float(rec.get('lat', 0.0)),
        lon=float(rec.get('lon', 0.0)),
        elevation_ft=float(rec.get('elevation', 0.0)),
    )


# ──────────────────────────────────────────────────────────────────────
# Geodesic helpers (small-distance approximation is fine for <50 nm)
# ──────────────────────────────────────────────────────────────────────

def offset_latlon(lat: float, lon: float, bearing_deg: float, dist_m: float) -> tuple[float, float]:
    """Return (lat2, lon2) offset by ``dist_m`` meters along ``bearing_deg`` true.

    Uses spherical-Earth forward formula (accurate to a few meters at
    glide-footprint distances).
    """
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    brg = math.radians(bearing_deg)
    d_r = dist_m / _EARTH_R_M

    lat2 = math.asin(
        math.sin(lat1) * math.cos(d_r)
        + math.cos(lat1) * math.sin(d_r) * math.cos(brg)
    )
    lon2 = lon1 + math.atan2(
        math.sin(brg) * math.sin(d_r) * math.cos(lat1),
        math.cos(d_r) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


# ──────────────────────────────────────────────────────────────────────
# Glide footprint geometry
# ──────────────────────────────────────────────────────────────────────

def glide_radius_m(altitude_agl_ft: float, ld_ratio: float) -> float:
    """Still-air glide range in meters from ``altitude_agl_ft`` at L/D ratio."""
    if altitude_agl_ft <= 0 or ld_ratio <= 0:
        return 0.0
    range_ft = altitude_agl_ft * ld_ratio
    return range_ft / _FT_PER_M


def estimate_failure_point(
    airport_lat: float,
    airport_lon: float,
    runway_heading_deg: float,
    altitude_agl_ft: float,
    climb_gradient_deg: float = 5.0,
    liftoff_distance_ft: float = 0.0,
) -> tuple[float, float]:
    """Estimate the lat/lon where the aircraft reaches ``altitude_agl_ft``.

    Assumes liftoff at ``liftoff_distance_ft`` past the airport reference
    point along the runway heading, then climbs at ``climb_gradient_deg``
    above horizontal.
    """
    if altitude_agl_ft <= 0:
        return airport_lat, airport_lon
    grad = max(0.5, climb_gradient_deg)
    dist_along_climb_ft = altitude_agl_ft / math.tan(math.radians(grad))
    total_ft = liftoff_distance_ft + dist_along_climb_ft
    return offset_latlon(
        airport_lat, airport_lon, runway_heading_deg, total_ft / _FT_PER_M
    )


def wind_drift_m(
    altitude_agl_ft: float,
    ld_ratio: float,
    best_glide_kias: float,
    wind_speed_kt: float,
) -> float:
    """Approx downwind drift while gliding from altitude to ground (m)."""
    if altitude_agl_ft <= 0 or best_glide_kias <= 0:
        return 0.0
    # Sink rate at L/D max: V / (L/D), in same units. Time = alt / sink.
    # alt_ft / (V_kt * 1.6878 / ld_ratio)   gives seconds (since 1 kt = 1.6878 ft/s)
    sink_fps = best_glide_kias * 1.6878 / max(ld_ratio, 1e-6)
    glide_time_s = altitude_agl_ft / max(sink_fps, 0.1)
    drift_ft = wind_speed_kt * 1.6878 * glide_time_s
    return drift_ft / _FT_PER_M


# ──────────────────────────────────────────────────────────────────────
# OpenStreetMap landing-candidate query (Overpass API)
# ──────────────────────────────────────────────────────────────────────

# What we look for. Each entry: (overpass query fragment, color, label,
# rating) where rating is one of 'good', 'caution', 'avoid'.
_OSM_QUERIES = [
    # Best — dedicated aviation surfaces
    ('node["aeroway"~"aerodrome|airstrip|airfield"]',           '#22c55e', 'Other airport / airstrip', 'good'),
    ('way["aeroway"~"aerodrome|airstrip|airfield|runway"]',     '#22c55e', 'Other airport / airstrip', 'good'),
    # Open agricultural land
    ('way["landuse"="farmland"]',                                '#84cc16', 'Farmland',                 'good'),
    ('way["landuse"="meadow"]',                                  '#84cc16', 'Meadow',                   'good'),
    ('way["landuse"="grass"]',                                   '#a3e635', 'Grass',                    'good'),
    ('way["leisure"="golf_course"]',                             '#16a34a', 'Golf course',              'good'),
    # Caution — flat but with hazards (wires, traffic, fences)
    ('way["highway"~"motorway|trunk|primary"]',                  '#f59e0b', 'Major road (caution: wires/traffic)', 'caution'),
    ('way["amenity"="parking"]',                                 '#f59e0b', 'Parking lot (caution: light poles)', 'caution'),
    # Avoid — water, dense built-up
    ('way["natural"="water"]',                                   '#dc2626', 'Water (avoid)',            'avoid'),
    ('way["waterway"~"river|canal"]',                            '#dc2626', 'River/canal (avoid)',      'avoid'),
    ('way["landuse"~"residential|industrial|commercial|retail"]','#dc2626', 'Built-up (avoid)',         'avoid'),
    ('way["natural"="wood"]',                                    '#dc2626', 'Forest (avoid)',           'avoid'),
]


def fetch_landing_candidates(
    lat: float,
    lon: float,
    radius_m: float,
    timeout_s: float = 25.0,
) -> dict:
    """Query Overpass API for landing-area candidates within ``radius_m``.

    Returns a dict ``{ 'features': [ {label, color, rating, geometry, tags} ], 'error': str|None }``
    where geometry is a list of (lat, lon) pairs (single point for nodes).
    """
    try:
        import requests
    except ImportError:
        return {'features': [], 'error': 'requests not installed'}

    # Cap radius to keep queries reasonable (Overpass enforces limits anyway)
    r = max(200.0, min(radius_m, 25_000.0))

    parts = []
    for query_frag, _, _, _ in _OSM_QUERIES:
        # Insert the around: filter
        # Overpass syntax: way["k"="v"](around:R,lat,lon);
        if query_frag.startswith('node'):
            base, rest = 'node', query_frag[4:]
        elif query_frag.startswith('way'):
            base, rest = 'way', query_frag[3:]
        else:
            continue
        parts.append(f'{base}{rest}(around:{r:.0f},{lat:.6f},{lon:.6f});')

    overpass_q = (
        '[out:json][timeout:25];('
        + ''.join(parts)
        + ');out tags geom 200;'
    )

    try:
        resp = requests.post(
            'https://overpass-api.de/api/interpreter',
            data={'data': overpass_q},
            timeout=timeout_s,
            headers={'User-Agent': 'TurnbackSimulator/1.0 (forced-landing-analysis)'},
        )
        resp.raise_for_status()
        data = resp.json()
    except Exception as exc:  # network / parse / timeout
        return {'features': [], 'error': f'{type(exc).__name__}: {exc}'}

    # Index our queries to map result tags back to category
    def classify(tags: dict) -> tuple[str, str, str]:
        """Return (color, label, rating) for an element's tags."""
        if not tags:
            return ('#888', 'Unknown', 'caution')
        if tags.get('aeroway') in ('aerodrome', 'airstrip', 'airfield', 'runway'):
            return ('#22c55e', 'Airport / airstrip', 'good')
        if tags.get('landuse') == 'farmland':
            return ('#84cc16', 'Farmland', 'good')
        if tags.get('landuse') == 'meadow':
            return ('#84cc16', 'Meadow', 'good')
        if tags.get('landuse') == 'grass':
            return ('#a3e635', 'Grass', 'good')
        if tags.get('leisure') == 'golf_course':
            return ('#16a34a', 'Golf course', 'good')
        if tags.get('highway') in ('motorway', 'trunk', 'primary'):
            return ('#f59e0b', 'Major road (caution)', 'caution')
        if tags.get('amenity') == 'parking':
            return ('#f59e0b', 'Parking lot (caution)', 'caution')
        if tags.get('natural') == 'water':
            return ('#dc2626', 'Water (avoid)', 'avoid')
        if tags.get('waterway') in ('river', 'canal'):
            return ('#dc2626', 'River/canal (avoid)', 'avoid')
        if tags.get('landuse') in ('residential', 'industrial', 'commercial', 'retail'):
            return ('#dc2626', 'Built-up (avoid)', 'avoid')
        if tags.get('natural') == 'wood':
            return ('#dc2626', 'Forest (avoid)', 'avoid')
        return ('#888', 'Other', 'caution')

    features = []
    for el in data.get('elements', []):
        tags = el.get('tags', {})
        color, label, rating = classify(tags)
        if el['type'] == 'node':
            geom = [(el['lat'], el['lon'])]
        elif el['type'] == 'way' and 'geometry' in el:
            geom = [(p['lat'], p['lon']) for p in el['geometry']]
        else:
            continue
        if not geom:
            continue
        features.append({
            'label': label,
            'color': color,
            'rating': rating,
            'geometry': geom,
            'name': tags.get('name', ''),
            'is_polygon': el['type'] == 'way' and len(geom) >= 4
                          and geom[0] == geom[-1],
        })
    return {'features': features, 'error': None}


def fetch_airport_perimeter(
    lat: float,
    lon: float,
    search_radius_m: float = 4000.0,
    timeout_s: float = 15.0,
) -> dict:
    """Query Overpass for the OSM airport-perimeter polygon nearest ``(lat, lon)``.

    Returns ``{'polygon': [(lat, lon), ...], 'name': str, 'error': str|None}``.
    ``polygon`` is empty if no aerodrome way was found within the radius.

    Strategy: pull every ``aeroway=aerodrome`` *way* (closed polygon) within
    the search radius, then pick the one whose centroid is nearest the input
    coordinate.  Ignores point nodes — we want the actual fenced boundary.
    """
    try:
        import requests
    except ImportError:
        return {'polygon': [], 'name': '', 'error': 'requests not installed'}

    r = max(500.0, min(search_radius_m, 10_000.0))
    overpass_q = (
        '[out:json][timeout:15];'
        f'(way["aeroway"="aerodrome"](around:{r:.0f},{lat:.6f},{lon:.6f});'
        f' relation["aeroway"="aerodrome"](around:{r:.0f},{lat:.6f},{lon:.6f}););'
        'out tags geom 50;'
    )

    try:
        resp = requests.post(
            'https://overpass-api.de/api/interpreter',
            data={'data': overpass_q},
            timeout=timeout_s,
            headers={'User-Agent': 'TurnbackSimulator/1.0 (airport-perimeter)'},
        )
        resp.raise_for_status()
        data = resp.json()
    except Exception as exc:
        return {'polygon': [], 'name': '', 'error': f'{type(exc).__name__}: {exc}'}

    best = None
    best_dist = float('inf')
    for el in data.get('elements', []):
        tags = el.get('tags', {}) or {}
        if tags.get('aeroway') != 'aerodrome':
            continue

        # Collect candidate rings: ways have 'geometry'; relations have 'members'
        rings = []
        if el.get('type') == 'way' and el.get('geometry'):
            rings.append([(p['lat'], p['lon']) for p in el['geometry']])
        elif el.get('type') == 'relation':
            for m in el.get('members', []):
                if m.get('type') == 'way' and m.get('role') in ('outer', '') and m.get('geometry'):
                    rings.append([(p['lat'], p['lon']) for p in m['geometry']])

        for geom in rings:
            if len(geom) < 4:
                continue
            # Centroid
            clat = sum(p[0] for p in geom) / len(geom)
            clon = sum(p[1] for p in geom) / len(geom)
            # Cheap planar distance (good enough for "nearest" within a few km)
            d = math.hypot((clat - lat) * 111_000, (clon - lon) * 111_000 * math.cos(math.radians(lat)))
            if d < best_dist:
                best_dist = d
                best = (geom, tags.get('name', '') or tags.get('icao', '') or tags.get('iata', ''))

    if best is None:
        return {'polygon': [], 'name': '', 'error': None}
    geom, name = best
    return {'polygon': geom, 'name': name, 'error': None}


# ──────────────────────────────────────────────────────────────────────
# Folium map builder
# ──────────────────────────────────────────────────────────────────────

def build_satellite_map(
    airport: Airport,
    runway_heading_deg: float,
    failure_point_low: tuple[float, float],
    failure_point_high: tuple[float, float],
    glide_radius_low_m: float,
    glide_radius_high_m: float,
    dead_zone_low_ft: float,
    dead_zone_high_ft: float,
    wind_from_deg: float = 0.0,
    wind_speed_kt: float = 0.0,
    candidates: Optional[list] = None,
    envelope_tracks: Optional[list] = None,
    airport_perimeter: Optional[list] = None,
):
    """Build a folium map centered on the airport with glide-footprint overlays.

    Returns the folium Map object (caller renders via streamlit_folium).

    ``envelope_tracks`` (Charlie #G4): optional list of dicts
        {label, color, weight, latlons: [(lat, lon), ...], tooltip}
    rendered as polylines on top of the satellite imagery.  Use to overlay
    the actual heart-shaped turnback ground tracks at the critical altitude.
    """
    import folium

    # Center between airport and the high-altitude failure point
    center_lat = (airport.lat + failure_point_high[0]) / 2
    center_lon = (airport.lon + failure_point_high[1]) / 2

    # Pick a starting zoom based on the larger glide footprint
    # ~150 m/px at zoom 11, ~75 m/px at 12, etc.; pick so circle fits.
    r = max(glide_radius_high_m, 1000.0)
    if r > 15000:
        zoom = 10
    elif r > 7000:
        zoom = 11
    elif r > 3500:
        zoom = 12
    elif r > 1800:
        zoom = 13
    else:
        zoom = 14

    fmap = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=zoom,
        tiles=None,
        control_scale=True,
    )

    # Esri World Imagery — default basemap, named cleanly for the layer
    # control (passing the raw URL via folium.Map(tiles=...) makes the URL
    # show up as the layer label, which is ugly).
    folium.TileLayer(
        tiles=('https://server.arcgisonline.com/ArcGIS/rest/services/'
               'World_Imagery/MapServer/tile/{z}/{y}/{x}'),
        attr='Tiles © Esri — Source: Esri, Maxar, Earthstar Geographics, '
             'and the GIS User Community',
        name='Satellite',
        max_zoom=19,
        overlay=False,
        control=True,
    ).add_to(fmap)

    # Alternate basemap available via LayerControl (Esri remains default).
    folium.TileLayer(
        'OpenStreetMap',
        name='OpenStreetMap',
        overlay=False,
        control=True,
    ).add_to(fmap)

    # Airport perimeter polygon (Charlie #F7-v2) — OSM aeroway=aerodrome.
    # Drawn BEFORE the dead-zone circles so they overlay it visibly.
    if airport_perimeter and len(airport_perimeter) >= 3:
        folium.Polygon(
            locations=airport_perimeter,
            color='#22c55e',
            weight=3,
            opacity=0.9,
            fill=True,
            fill_color='#22c55e',
            fill_opacity=0.18,
            tooltip='Airport perimeter (OSM) — straight-ahead landing zone',
            popup=('<b>Airport perimeter</b><br>'
                   'Below straight-ahead-max altitude, anywhere inside this '
                   'green boundary is survivable.  Source: OpenStreetMap '
                   '(aeroway=aerodrome).'),
        ).add_to(fmap)

    # Airport perimeter polygon (Charlie #F7-v2) — OSM aeroway=aerodrome.
    # Drawn BEFORE the airport marker / circles so they overlay it visibly.
    if airport_perimeter and len(airport_perimeter) >= 3:
        folium.Polygon(
            locations=airport_perimeter,
            color='#22c55e',
            weight=3,
            opacity=0.9,
            fill=True,
            fill_color='#22c55e',
            fill_opacity=0.18,
            tooltip='Airport perimeter (OSM) — straight-ahead landing zone',
            popup=('<b>Airport perimeter</b><br>'
                   'Below straight-ahead-max altitude, anywhere inside this '
                   'green boundary is survivable.  Source: OpenStreetMap '
                   '(aeroway=aerodrome).'),
        ).add_to(fmap)

    # Airport marker
    folium.Marker(
        location=[airport.lat, airport.lon],
        popup=folium.Popup(
            f"<b>{airport.icao or airport.iata or airport.code}</b><br>"
            f"{airport.name}<br>"
            f"{airport.city}, {airport.country}<br>"
            f"Elev {airport.elevation_ft:.0f} ft",
            max_width=250,
        ),
        tooltip=airport.icao or airport.iata or airport.code,
        icon=folium.Icon(color='blue', icon='plane', prefix='fa'),
    ).add_to(fmap)

    # Runway centerline (extend ~3 nm in departure direction)
    end_lat, end_lon = offset_latlon(
        airport.lat, airport.lon, runway_heading_deg, 3 * _M_PER_NM
    )
    folium.PolyLine(
        [(airport.lat, airport.lon), (end_lat, end_lon)],
        color='#fbbf24',
        weight=3,
        opacity=0.7,
        tooltip=f"Departure heading {runway_heading_deg:.0f}°",
    ).add_to(fmap)

    # Climb-out path (magenta) — from the airport reference point along the
    # runway heading out to the engine-failure point at the turnback critical
    # altitude.  This is the line you fly during a normal departure up to the
    # moment the engine quits.
    if failure_point_high and dead_zone_high_ft > 0:
        folium.PolyLine(
            [(airport.lat, airport.lon), tuple(failure_point_high)],
            color='#ff00ff',
            weight=5,
            opacity=0.9,
            tooltip=(f"Climb-out path → engine fails at "
                     f"{dead_zone_high_ft:.0f} ft AGL (turnback critical)"),
            popup=folium.Popup(
                f"<b>Climb-out path</b><br>"
                f"Lift-off → engine failure at the turnback critical "
                f"altitude ({dead_zone_high_ft:.0f} ft AGL).<br>"
                f"Length along ground reflects the climb gradient and "
                f"liftoff distance.",
                max_width=260,
            ),
        ).add_to(fmap)
        # Small magenta marker at the lift-off / brake-release end so the
        # user can see the path origin.
        folium.CircleMarker(
            location=[airport.lat, airport.lon],
            radius=4, color='#ff00ff', fill=True, fill_opacity=1.0,
            tooltip="Brake release / lift-off",
        ).add_to(fmap)

    # Glide footprint at LOW edge of dead zone (just above straight-ahead max)
    if glide_radius_low_m > 0 and dead_zone_low_ft > 0:
        folium.Circle(
            location=list(failure_point_low),
            radius=glide_radius_low_m,
            color='#f87171',
            weight=2,
            fill=True,
            fill_opacity=0.12,
            popup=(f"Glide reach @ {dead_zone_low_ft:.0f} ft AGL "
                   f"(low end of critical zone) — radius "
                   f"{glide_radius_low_m / _M_PER_NM:.2f} nm"),
            tooltip=f"Critical-zone LOW: {dead_zone_low_ft:.0f} ft AGL",
        ).add_to(fmap)
        folium.CircleMarker(
            location=list(failure_point_low),
            radius=4, color='#f87171', fill=True,
            tooltip=f"Failure point @ {dead_zone_low_ft:.0f} ft",
        ).add_to(fmap)

    # Glide footprint at HIGH edge of dead zone (turnback critical altitude)
    if glide_radius_high_m > 0:
        folium.Circle(
            location=list(failure_point_high),
            radius=glide_radius_high_m,
            color='#fbbf24',
            weight=2,
            fill=True,
            fill_opacity=0.10,
            popup=(f"Glide reach @ {dead_zone_high_ft:.0f} ft AGL "
                   f"(top of critical zone / turnback critical) — radius "
                   f"{glide_radius_high_m / _M_PER_NM:.2f} nm"),
            tooltip=f"Critical-zone HIGH: {dead_zone_high_ft:.0f} ft AGL",
        ).add_to(fmap)
        folium.CircleMarker(
            location=list(failure_point_high),
            radius=4, color='#fbbf24', fill=True,
            tooltip=f"Failure point @ {dead_zone_high_ft:.0f} ft",
        ).add_to(fmap)

    # Wind arrow (from departure end, pointing TO the wind source)
    if wind_speed_kt > 0:
        wind_lat, wind_lon = offset_latlon(
            airport.lat, airport.lon, wind_from_deg, 1500.0
        )
        folium.PolyLine(
            [(airport.lat, airport.lon), (wind_lat, wind_lon)],
            color='#60a5fa', weight=2, opacity=0.8, dash_array='6,6',
            tooltip=f"Wind from {wind_from_deg:.0f}° @ {wind_speed_kt:.0f} kt",
        ).add_to(fmap)

    # Heart-shape envelope tracks (Charlie #G4)
    if envelope_tracks:
        env_group = folium.FeatureGroup(name='Turnback ground tracks', show=True)
        for trk in envelope_tracks:
            latlons = trk.get('latlons') or []
            if len(latlons) < 2:
                continue
            folium.PolyLine(
                latlons,
                color=trk.get('color', '#facc15'),
                weight=trk.get('weight', 3),
                opacity=0.85,
                dash_array=trk.get('dash_array'),
                tooltip=trk.get('tooltip') or trk.get('label'),
                popup=trk.get('label'),
            ).add_to(env_group)
        env_group.add_to(fmap)

    # Landing candidates from OSM
    if candidates:
        good = folium.FeatureGroup(name='Good landing areas', show=True)
        caution = folium.FeatureGroup(name='Caution', show=True)
        avoid = folium.FeatureGroup(name='Avoid', show=False)
        groups = {'good': good, 'caution': caution, 'avoid': avoid}

        for feat in candidates:
            grp = groups.get(feat['rating'], caution)
            popup_text = (f"<b>{feat['label']}</b>"
                          + (f"<br>{feat['name']}" if feat['name'] else ''))
            if feat['is_polygon']:
                folium.Polygon(
                    locations=feat['geometry'],
                    color=feat['color'], weight=1,
                    fill=True, fill_opacity=0.35,
                    popup=popup_text,
                    tooltip=feat['label'],
                ).add_to(grp)
            elif len(feat['geometry']) > 1:
                folium.PolyLine(
                    feat['geometry'],
                    color=feat['color'], weight=2, opacity=0.8,
                    popup=popup_text, tooltip=feat['label'],
                ).add_to(grp)
            else:
                folium.CircleMarker(
                    location=feat['geometry'][0],
                    radius=5, color=feat['color'],
                    fill=True, fill_opacity=0.8,
                    popup=popup_text, tooltip=feat['label'],
                ).add_to(grp)

        good.add_to(fmap)
        caution.add_to(fmap)
        avoid.add_to(fmap)

    # ── Legend overlay (May 2026) ──
    # Builds a small HTML panel in the bottom-left listing every visible
    # track style so pilots can read the satellite map without guessing.
    _legend_rows = []
    if envelope_tracks:
        for trk in envelope_tracks:
            color = trk.get('color', '#facc15')
            dash = trk.get('dash_array')
            label = trk.get('label', '—')
            stroke = (
                f"border-top: 3px dashed {color};"
                if dash else
                f"border-top: 3px solid {color};"
            )
            _legend_rows.append(
                f"<div style='display:flex;align-items:center;margin:2px 0;'>"
                f"<span style='display:inline-block;width:30px;height:0;{stroke}"
                f"margin-right:8px;'></span>"
                f"<span style='font-size:11px;'>{label}</span></div>"
            )
    if wind_speed_kt > 0:
        _legend_rows.append(
            "<div style='display:flex;align-items:center;margin:2px 0;'>"
            "<span style='display:inline-block;width:30px;height:0;"
            "border-top: 2px dashed #60a5fa;margin-right:8px;'></span>"
            f"<span style='font-size:11px;'>Wind from {wind_from_deg:.0f}° "
            f"@ {wind_speed_kt:.0f} kt</span></div>"
        )
    if _legend_rows:
        _legend_html = (
            "<details style='position: fixed; bottom: 30px; left: 10px; z-index: 9999; "
            "background: rgba(255,255,255,0.92); padding: 4px 8px; "
            "border: 1px solid #999; border-radius: 4px; "
            "font-family: sans-serif; max-width: 320px; "
            "box-shadow: 0 1px 4px rgba(0,0,0,0.2);'>"
            "<summary style='font-weight:bold;font-size:11px;cursor:pointer;"
            "list-style:revert;'>Map legend</summary>"
            "<div style='margin-top:4px;'>"
            + "".join(_legend_rows) +
            "</div></details>"
        )
        fmap.get_root().html.add_child(folium.Element(_legend_html))

    folium.LayerControl(collapsed=True).add_to(fmap)
    return fmap


# ──────────────────────────────────────────────────────────────────────
# 3-D satellite map (pydeck) — Charlie #G2
# ──────────────────────────────────────────────────────────────────────

def build_3d_satellite_map(
    airport: Airport,
    runway_heading_deg: float,
    envelope_tracks_3d=None,
    airport_perimeter=None,
    altitude_exaggeration: float = 5.0,
    pitch_deg: float = 55.0,
    bearing_deg: float = 0.0,
    glide_radius_high_m: float = 0.0,
):
    """Return a ``pydeck.Deck`` rendering the turnback ground tracks
    in 3-D over satellite imagery (Esri World Imagery, no token needed).

    Args:
        envelope_tracks_3d: list of dicts with keys ``label``, ``color``
            (R,G,B int 0-255), and ``coords`` = list of ``[lon, lat, alt_m]``.
        altitude_exaggeration: multiplier applied to altitude in meters
            so the climb/turnback profile is visible at airport scale
            (default 5×).  Shown in the legend.
        pitch_deg / bearing_deg: initial camera angles.
    """
    import pydeck as pdk

    layers = []

    # Esri World Imagery basemap — free, no API key.
    layers.append(pdk.Layer(
        "TileLayer",
        data="https://server.arcgisonline.com/ArcGIS/rest/services/"
             "World_Imagery/MapServer/tile/{z}/{y}/{x}",
        min_zoom=0,
        max_zoom=19,
        tile_size=256,
        opacity=1.0,
    ))

    # Airport perimeter as a green polygon at field elevation.
    if airport_perimeter:
        peri_coords = [[lon, lat] for lat, lon in airport_perimeter]
        if peri_coords and peri_coords[0] != peri_coords[-1]:
            peri_coords.append(peri_coords[0])
        layers.append(pdk.Layer(
            "PolygonLayer",
            data=[{"polygon": peri_coords}],
            get_polygon="polygon",
            get_fill_color=[34, 197, 94, 40],
            get_line_color=[22, 163, 74, 220],
            line_width_min_pixels=2,
            stroked=True,
            filled=True,
            pickable=False,
        ))

    # Runway centerline arrow (1.5 nm projection).
    end_lat, end_lon = offset_latlon(
        airport.lat, airport.lon, runway_heading_deg, 2780.0  # ~1.5 nm
    )
    layers.append(pdk.Layer(
        "PathLayer",
        data=[{"path": [
            [airport.lon, airport.lat, 0.0],
            [end_lon, end_lat, 0.0],
        ], "color": [250, 204, 21]}],
        get_path="path",
        get_color="color",
        get_width=4,
        width_min_pixels=2,
        pickable=False,
    ))

    # Turnback / straight-ahead 3-D paths.
    if envelope_tracks_3d:
        path_records = []
        for tr in envelope_tracks_3d:
            coords = tr.get("coords") or []
            if len(coords) < 2:
                continue
            # Apply altitude exaggeration to z (third element).
            scaled = [
                [c[0], c[1], (c[2] if len(c) > 2 else 0.0) * altitude_exaggeration]
                for c in coords
            ]
            base_color = list(tr.get("color", (34, 197, 94)))
            alpha = int(tr.get("alpha", 255))
            rgba = base_color[:3] + [alpha]
            path_records.append({
                "path": scaled,
                "color": rgba,
                "width": float(tr.get("width", 5)),
                "name": tr.get("label", ""),
            })
        if path_records:
            layers.append(pdk.Layer(
                "PathLayer",
                data=path_records,
                get_path="path",
                get_color="color",
                get_width="width",
                width_min_pixels=2,
                pickable=True,
            ))

    # Airport reference point as a vertical column to anchor the eye.
    layers.append(pdk.Layer(
        "ColumnLayer",
        data=[{"position": [airport.lon, airport.lat], "value": 50.0}],
        get_position="position",
        get_elevation="value",
        elevation_scale=1.0,
        radius=15,
        get_fill_color=[59, 130, 246, 220],
        pickable=False,
    ))

    # Camera: zoom chosen so glide footprint is visible.
    radius_m = max(glide_radius_high_m, 800.0)
    # Empirical: at 60° latitude-independent zoom, ~radius_m/2 fits view.
    zoom = max(11.5, min(15.5, 16.0 - math.log2(max(radius_m / 800.0, 1.0))))

    view_state = pdk.ViewState(
        latitude=airport.lat,
        longitude=airport.lon,
        zoom=zoom,
        pitch=pitch_deg,
        bearing=bearing_deg,
    )

    return pdk.Deck(
        layers=layers,
        initial_view_state=view_state,
        map_style=None,  # disable default Mapbox basemap; TileLayer above wins
        tooltip={"text": "{name}"},
    )


# ──────────────────────────────────────────────────────────────────────
# Streamlit section
# ──────────────────────────────────────────────────────────────────────

def render_landing_map_section(
    *,
    critical_alt_low_ft: float,
    critical_alt_high_ft: float,
    straight_ahead_max_alt_ft: float,
    ld_ratio: float,
    best_glide_kias: float,
    wind_speed_kt: float = 0.0,
    wind_from_deg: float = 0.0,
    liftoff_distance_ft: float = 0.0,
    default_airport_code: str = "KSEZ",
    default_runway_heading: float = 30.0,
    envelope=None,
    critical_alt: float = 0.0,
    straight_ahead_max_alt: float = 0.0,
    comparison_envelope=None,
    comparison_critical_alt: float = 0.0,
    comparison_label: str = None,
    primary_label: str = None,
):
    """Render the satellite-map / forced-landing analysis section in the UI.

    Parameters mirror what the simulator already computes. Call this
    after the envelope / runway-zone analysis section.

    ``envelope`` (Charlie #G4): if provided along with ``critical_alt``,
    the critical-altitude turnback ground tracks (left & right) and the
    straight-ahead-max ground track are overlaid on the satellite map.
    """
    import streamlit as st

    st.markdown("---")
    st.subheader("🛰️ Satellite Map — Forced-Landing Analysis")
    st.caption(
        "Visualize the **critical-zone glide footprint** at your chosen airport. "
        "The critical zone is the altitude band where you can't land on the "
        "remaining runway and can't make the turnback — these are the "
        "altitudes where picking a forced-landing site matters most."
    )

    # ── Inputs ──
    cols = st.columns([1, 1, 1, 1])
    _default_code = (default_airport_code or "KSEZ").strip().upper() or "KSEZ"
    code = cols[0].text_input(
        "Airport code (ICAO or IATA)", value=_default_code,
        help="ICAO 4-letter (e.g. KSEZ, EGLL) or IATA 3-letter (e.g. SDX, LHR)",
    ).strip().upper()
    _default_hdg = int(round(default_runway_heading)) % 360 if default_runway_heading else 30
    runway_heading = cols[1].number_input(
        "Departure runway heading (° true)",
        min_value=0, max_value=359, value=_default_hdg, step=5,
        help="The magnetic/true heading you depart on. Used to position "
             "the glide footprint along the climb-out path. Pre-filled from "
             "the runway selected in the sidebar.",
    )
    climb_gradient = cols[2].number_input(
        "Climb gradient (°)",
        min_value=1.0, max_value=15.0, value=5.0, step=0.5,
        help="Typical light-single climb gradient. Used to estimate where "
             "the aircraft is when the engine quits at each altitude.",
    )
    fetch_osm = cols[3].checkbox(
        "Find landing candidates (OSM)", value=True,
        help="Query OpenStreetMap for nearby fields, golf courses, water, "
             "and built-up areas within glide range. Requires internet; "
             "may take a few seconds.",
    )
    show_perimeter = st.checkbox(
        "Show airport perimeter (OSM)", value=True,
        help="Overlay the OpenStreetMap aerodrome boundary as a green polygon. "
             "This is the actual fenced area inside which a straight-ahead "
             "landing is survivable \u2014 *not* just the runway asphalt.",
    )

    if not code:
        st.info("Enter an airport code to see the satellite map.")
        return

    airport = lookup_airport(code)
    if airport is None:
        st.warning(
            f"Airport `{code}` not found. Try the ICAO code (e.g. `KSEZ` for "
            "Sedona, `KORD` for Chicago O'Hare, `EGLL` for Heathrow)."
        )
        return

    # ── Dead-zone band ──
    dead_low = max(0.0, straight_ahead_max_alt_ft)
    dead_high = max(critical_alt_low_ft, critical_alt_high_ft)
    # Charlie #F7 — straight-ahead boundary is the *airport perimeter*, not
    # just runway asphalt.  Make this explicit on the map page.
    st.info(
        "**Straight-ahead landing zone = the entire airport boundary, "
        "not just the runway asphalt.**  Below the straight-ahead-max "
        f"altitude ({int(round(straight_ahead_max_alt_ft)):,} ft AGL), aim for any "
        "open area inside the airport fence — taxiway, infield grass, "
        "ramp, even a parking lot — *anything but a turn back to the "
        "departure runway*.  An overrun onto airport infield is survivable; "
        "a stall-spin in the turnback is not.  *(The green polygon on the "
        "map below is the actual OSM airport perimeter.)*"
    )
    if dead_high <= dead_low:
        st.success(
            "✅ **No critical zone with current parameters.** Straight-ahead "
            "landing coverage extends to or above the turnback critical "
            "altitude. The glide footprint shown is at the turnback "
            "critical altitude only."
        )
        # Still show a single ring at the turnback critical alt
        dead_low = dead_high
    elif dead_high <= 0:
        st.info("Build the envelope first to compute the critical zone.")
        return

    # ── Failure points & glide radii ──
    fp_low = estimate_failure_point(
        airport.lat, airport.lon, runway_heading, dead_low,
        climb_gradient, liftoff_distance_ft,
    )
    fp_high = estimate_failure_point(
        airport.lat, airport.lon, runway_heading, dead_high,
        climb_gradient, liftoff_distance_ft,
    )
    r_low_m = glide_radius_m(dead_low, ld_ratio)
    r_high_m = glide_radius_m(dead_high, ld_ratio)

    # ── Stats ──
    info_cols = st.columns(4)
    info_cols[0].metric(
        "Airport",
        airport.icao or airport.iata or airport.code,
        f"{airport.name[:30]}",
    )
    info_cols[1].metric(
        "Critical-zone band",
        f"{dead_low:,.0f} – {dead_high:,.0f} ft",
        f"{(dead_high - dead_low):,.0f} ft band",
    )
    info_cols[2].metric(
        "Glide reach (low)",
        f"{r_low_m / _M_PER_NM:.2f} nm",
        f"@ {dead_low:,.0f} ft, L/D {ld_ratio:.1f}",
    )
    info_cols[3].metric(
        "Glide reach (high)",
        f"{r_high_m / _M_PER_NM:.2f} nm",
        f"@ {dead_high:,.0f} ft, L/D {ld_ratio:.1f}",
    )

    if wind_speed_kt > 0:
        drift_low = wind_drift_m(dead_low, ld_ratio, best_glide_kias, wind_speed_kt)
        drift_high = wind_drift_m(dead_high, ld_ratio, best_glide_kias, wind_speed_kt)
        st.caption(
            f"⚠️ Wind {wind_speed_kt:.0f} kt from {wind_from_deg:.0f}° "
            f"will drift the footprint downwind by ~"
            f"{drift_low / _M_PER_NM:.2f} nm (low) / "
            f"{drift_high / _M_PER_NM:.2f} nm (high). "
            f"The circles below are still-air; mentally bias the reachable "
            f"area downwind."
        )

    # ── Optional: fetch OSM landing candidates ──
    candidates = None
    if fetch_osm:
        with st.spinner("Querying OpenStreetMap for landing candidates..."):
            result = fetch_landing_candidates(
                fp_high[0], fp_high[1], r_high_m,
            )
        if result['error']:
            st.warning(f"OSM query failed: {result['error']}")
        else:
            candidates = result['features']
            st.caption(
                f"Found **{len(candidates)}** features within glide range. "
                f"Toggle layers via the control on the map."
            )
            # Charlie #G5 — color swatches next to good/caution/avoid so the
            # map's coloring is self-explanatory without hovering features.
            st.markdown(
                """
                <div style='display:flex;gap:18px;flex-wrap:wrap;font-size:13px;
                            margin:6px 0 10px 0;'>
                  <span><span style='display:inline-block;width:14px;height:14px;
                        background:#22c55e;border-radius:3px;vertical-align:middle;
                        margin-right:6px;'></span><strong>GOOD</strong> — airports,
                        farmland, meadow, golf course</span>
                  <span><span style='display:inline-block;width:14px;height:14px;
                        background:#f59e0b;border-radius:3px;vertical-align:middle;
                        margin-right:6px;'></span><strong>CAUTION</strong> — major
                        roads, parking lots (wires, traffic, light poles)</span>
                  <span><span style='display:inline-block;width:14px;height:14px;
                        background:#dc2626;border-radius:3px;vertical-align:middle;
                        margin-right:6px;'></span><strong>AVOID</strong> — water,
                        rivers, dense built-up</span>
                </div>
                """,
                unsafe_allow_html=True,
            )

    # ── Render the map ──
    try:
        from streamlit_folium import st_folium
    except ImportError:
        st.error(
            "`streamlit-folium` is not installed. Add it to requirements.txt "
            "and rebuild the container."
        )
        return

    # ── G4: build heart-shape ground-track polylines from envelope ──
    envelope_tracks = None
    envelope_tracks_3d = None  # G2: parallel 3-D structure for pydeck
    if envelope:
        envelope_tracks = []
        envelope_tracks_3d = []
        # Pick the envelope row at (or just above) critical_alt for each side
        target_alt = max(critical_alt, max(critical_alt_low_ft, critical_alt_high_ft))
        rwy_hdg = runway_heading

        def _track_to_latlons(traj):
            """Convert (x_ft, y_ft) trajectory to [(lat, lon), ...].
            x = lateral (right of centerline +), y = downrange along runway hdg.
            """
            pts = []
            for p in traj:
                x_ft = p.get('x', 0.0)
                y_ft = p.get('y', 0.0)
                dist_ft = math.hypot(x_ft, y_ft)
                if dist_ft < 1e-6:
                    pts.append((airport.lat, airport.lon))
                    continue
                # Bearing offset from runway centerline: +x to the right
                bearing = (rwy_hdg + math.degrees(math.atan2(x_ft, y_ft))) % 360.0
                lat2, lon2 = offset_latlon(
                    airport.lat, airport.lon, bearing, dist_ft / _FT_PER_M
                )
                pts.append((lat2, lon2))
            return pts

        def _track_to_lonlatalt(traj):
            """Same as _track_to_latlons but also carries z (altitude AGL ft)
            converted to meters.  Returns [[lon, lat, alt_m], ...].
            Used by the 3-D pydeck map (Charlie #G2).
            """
            pts = []
            for p in traj:
                x_ft = p.get('x', 0.0)
                y_ft = p.get('y', 0.0)
                z_ft = max(p.get('z', 0.0), 0.0)
                dist_ft = math.hypot(x_ft, y_ft)
                if dist_ft < 1e-6:
                    pts.append([airport.lon, airport.lat, z_ft / _FT_PER_M])
                    continue
                bearing = (rwy_hdg + math.degrees(math.atan2(x_ft, y_ft))) % 360.0
                lat2, lon2 = offset_latlon(
                    airport.lat, airport.lon, bearing, dist_ft / _FT_PER_M
                )
                pts.append([lon2, lat2, z_ft / _FT_PER_M])
            return pts

        # Find the per-side critical altitude (lowest alt at which that side
        # succeeds).  Crosswind makes left vs right asymmetric, so we want
        # the legend to show DIFFERENT altitudes for each side rather than
        # collapsing to a single shared row.
        crit_row_per_side = {'left': None, 'right': None}
        for row in envelope:
            for side in ('left', 'right'):
                if crit_row_per_side[side] is not None:
                    continue
                sub = row.get(side) or {}
                if sub.get('success') and sub.get('trajectory'):
                    crit_row_per_side[side] = row

        if crit_row_per_side['left'] or crit_row_per_side['right']:
            _prim = primary_label or "Turnback"
            _primary_climb_endpoint = None  # captured below for the 3-D climb line
            for side, color, label in (
                ('left',  '#16a34a', f'{_prim} LEFT turn'),
                ('right', '#22c55e', f'{_prim} RIGHT turn'),
            ):
                row = crit_row_per_side[side]
                if not row:
                    continue
                sub = row.get(side) or {}
                if sub.get('success') and sub.get('trajectory'):
                    _coords3d = _track_to_lonlatalt(sub['trajectory'])
                    if _coords3d and _primary_climb_endpoint is None:
                        _primary_climb_endpoint = _coords3d[0]
                    envelope_tracks.append({
                        'label': f"{label} @ {row['alt_agl']} ft AGL",
                        'color': color,
                        'weight': 4,
                        'latlons': _track_to_latlons(sub['trajectory']),
                        'tooltip': f"{label} from {row['alt_agl']} ft AGL",
                    })
                    envelope_tracks_3d.append({
                        'label': f"{label} @ {row['alt_agl']} ft AGL",
                        'color': (22, 163, 74) if side == 'left' else (34, 197, 94),
                        'coords': _coords3d,
                    })

            # Magenta climb-out path (3-D only) — airport ground level up
            # to the engine-failure point at critical altitude. Mirrors the
            # 2-D magenta climb line. (May 2026)
            if _primary_climb_endpoint is not None:
                envelope_tracks_3d.append({
                    'label': f"{_prim} climb-out",
                    'color': (255, 0, 255),
                    'width': 7,
                    'alpha': 255,
                    'coords': [
                        [airport.lon, airport.lat, 0.0],
                        _primary_climb_endpoint,
                    ],
                })

        # ── E2-P2: comparison-envelope tracks (alternate climb steering) ──
        # Per-side critical altitude (same asymmetric-crosswind fix as primary).
        if comparison_envelope and comparison_critical_alt > 0:
            cmp_label = comparison_label or "Alternate climb steering"
            cmp_row_per_side = {'left': None, 'right': None}
            for row in comparison_envelope:
                for side in ('left', 'right'):
                    if cmp_row_per_side[side] is not None:
                        continue
                    sub = row.get(side) or {}
                    if sub.get('success') and sub.get('trajectory'):
                        cmp_row_per_side[side] = row

            _cmp_climb_endpoint = None
            for side, label_side in (('left', 'LEFT'), ('right', 'RIGHT')):
                cmp_row = cmp_row_per_side[side]
                if not cmp_row:
                    continue
                sub = cmp_row.get(side) or {}
                if sub.get('success') and sub.get('trajectory'):
                    _coords3d_cmp = _track_to_lonlatalt(sub['trajectory'])
                    if _coords3d_cmp and _cmp_climb_endpoint is None:
                        _cmp_climb_endpoint = _coords3d_cmp[0]
                    envelope_tracks.append({
                        'label': f"{cmp_label} {label_side} @ {cmp_row['alt_agl']} ft AGL",
                        'color': '#facc15',  # gold
                        'weight': 3,
                        'dash_array': '8,6',
                        'latlons': _track_to_latlons(sub['trajectory']),
                        'tooltip': f"{cmp_label} {label_side} from "
                                   f"{cmp_row['alt_agl']} ft AGL",
                    })
                    envelope_tracks_3d.append({
                        'label': f"{cmp_label} {label_side} @ {cmp_row['alt_agl']} ft AGL",
                        'color': (250, 204, 21),
                        'coords': _coords3d_cmp,
                    })

            # Magenta-pink climb-out path for the alternate steering mode.
            if _cmp_climb_endpoint is not None:
                envelope_tracks_3d.append({
                    'label': f"{cmp_label} climb-out",
                    'color': (255, 0, 255),  # same magenta as primary
                    'width': 3,
                    'alpha': 130,             # thin + translucent = "alternate"
                    'coords': [
                        [airport.lon, airport.lat, 0.0],
                        _cmp_climb_endpoint,
                    ],
                })

            # Magenta-pink climb-out path for the alternate steering mode.
            try:
                _ep = _cmp_climb_endpoint
            except NameError:
                _ep = None
            if _ep is not None:
                envelope_tracks_3d.append({
                    'label': f"{cmp_label} climb-out",
                    'color': (236, 72, 153),  # pink — distinct from primary magenta
                    'coords': [
                        [airport.lon, airport.lat, 0.0],
                        _ep,
                    ],
                })

        # Straight-ahead-max track (engineless landing along centerline)
        sa_row = None
        if straight_ahead_max_alt > 0:
            for row in envelope:
                if (row.get('alt_agl', 0) >= straight_ahead_max_alt
                        and row.get('straight_ahead', {}).get('success')):
                    sa_row = row
                    break
        if sa_row and sa_row.get('straight_ahead', {}).get('trajectory'):
            envelope_tracks.append({
                'label': f"Straight-ahead max @ {sa_row['alt_agl']} ft AGL",
                'color': '#22d3ee',
                'weight': 3,
                'dash_array': '6,6',
                'latlons': _track_to_latlons(sa_row['straight_ahead']['trajectory']),
                'tooltip': f"Straight-ahead max @ {sa_row['alt_agl']} ft AGL",
            })
            envelope_tracks_3d.append({
                'label': f"Straight-ahead max @ {sa_row['alt_agl']} ft AGL",
                'color': (34, 211, 238),
                'coords': _track_to_lonlatalt(sa_row['straight_ahead']['trajectory']),
            })

        # ── E2-P4: Decision-ladder example tracks (May 2026) ──
        # Show one representative ground track per maneuver class so the
        # pilot can see what 180° vs 540° vs full-circuit actually looks
        # like over the satellite imagery.
        try:
            from analysis.turnback_simulator import find_min_alt_per_maneuver
            _kind_alts = find_min_alt_per_maneuver(envelope)
        except Exception:
            _kind_alts = {}
        # 180° already shown in solid green above; only add 540° and circuit
        # to avoid double-drawing the same heart shape.
        for _kind, _color_hex, _color_rgb, _label in (
            ('540', '#f59e0b', (245, 158, 11), '540° (orbit + opp rwy)'),
            ('circuit', '#a855f7', (168, 85, 247), 'Full circuit (same rwy)'),
        ):
            _info = _kind_alts.get(_kind)
            if not _info:
                continue
            _sub = _info.get('sub') or {}
            _traj = _sub.get('trajectory')
            if not _traj:
                continue
            envelope_tracks.append({
                'label': f"{_label} {_info['side'].upper()} @ {_info['alt']} ft AGL",
                'color': _color_hex,
                'weight': 3,
                'latlons': _track_to_latlons(_traj),
                'tooltip': f"{_label} from {_info['alt']} ft AGL "
                           f"({_info['side'].upper()} turn)",
            })
            envelope_tracks_3d.append({
                'label': f"{_label} {_info['side'].upper()} @ {_info['alt']} ft AGL",
                'color': _color_rgb,
                'coords': _track_to_lonlatalt(_traj),
            })

    # ── F7-v2: fetch OSM airport perimeter polygon (cached per session) ──
    airport_perimeter = None
    if show_perimeter:
        cache_key = f"_perimeter_cache_{airport.icao or airport.iata or code}"
        cached = st.session_state.get(cache_key)
        if cached is not None:
            airport_perimeter = cached or None
        else:
            with st.spinner("Fetching airport perimeter from OpenStreetMap…"):
                peri = fetch_airport_perimeter(airport.lat, airport.lon)
            if peri.get('error'):
                st.caption(f"⚠ OSM perimeter unavailable: {peri['error']}")
                st.session_state[cache_key] = []
            else:
                airport_perimeter = peri.get('polygon') or None
                st.session_state[cache_key] = airport_perimeter or []
                if not airport_perimeter:
                    st.caption("ℹ No `aeroway=aerodrome` polygon in OSM near this airport.")

    fmap = build_satellite_map(
        airport=airport,
        runway_heading_deg=runway_heading,
        failure_point_low=fp_low,
        failure_point_high=fp_high,
        glide_radius_low_m=r_low_m,
        glide_radius_high_m=r_high_m,
        dead_zone_low_ft=dead_low,
        dead_zone_high_ft=dead_high,
        wind_from_deg=wind_from_deg,
        wind_speed_kt=wind_speed_kt,
        candidates=candidates,
        envelope_tracks=envelope_tracks,
        airport_perimeter=airport_perimeter,
    )
    st_folium(fmap, height=600, width=None, returned_objects=[])

    # ── Charlie #G2: 3-D satellite map with turnback curves ──
    if envelope_tracks_3d:
        st.markdown("### 🛰️ 3-D Satellite View — Turnback Curves")
        ctl_cols = st.columns([1, 1, 1, 2])
        show_3d = ctl_cols[0].checkbox(
            "Show 3-D map", value=True,
            help=("Renders the critical-altitude turnback ground tracks in 3-D "
                  "over Esri World Imagery satellite tiles.  Drag to rotate; "
                  "shift-drag (or two-finger) to pitch."),
        )
        if show_3d:
            exag = ctl_cols[1].slider(
                "Vertical exaggeration",
                min_value=1.0, max_value=15.0, value=5.0, step=0.5,
                help=("Altitude multiplier applied to the 3-D paths.  At 1× "
                      "the climb profile is barely visible at airport scale; "
                      "5–10× makes the 'heart shape' easy to read."),
            )
            pitch = ctl_cols[2].slider(
                "Camera pitch (°)",
                min_value=0, max_value=85, value=55, step=5,
                help="0° = top-down, 60–70° = oblique.",
            )
            try:
                deck = build_3d_satellite_map(
                    airport=airport,
                    runway_heading_deg=runway_heading,
                    envelope_tracks_3d=envelope_tracks_3d,
                    airport_perimeter=airport_perimeter,
                    altitude_exaggeration=exag,
                    pitch_deg=float(pitch),
                    glide_radius_high_m=r_high_m,
                )
                st.pydeck_chart(deck, use_container_width=True)
                st.caption(
                    f"Altitudes are exaggerated **{exag:.1f}×**. Green = "
                    f"turnback ground track at the critical altitude "
                    f"({int(target_alt):,} ft AGL).  Cyan = straight-ahead "
                    f"max-altitude glide.  Tracks start at the engine-failure "
                    f"point above the runway and descend to touchdown."
                )
            except Exception as exc:  # noqa: BLE001 — pydeck failures are non-fatal
                st.warning(f"3-D map could not render: {exc}")

    # ── Interpretation guide ──
    with st.expander("How to read this map", expanded=False):
        st.markdown(f"""
- **Blue plane icon** — airport reference point (`{airport.icao or airport.code}`).
- **Yellow line** — runway departure centerline (heading {runway_heading:.0f}°).
- **Red dot + faint red circle** — engine failure at the **low end** of the
  critical zone ({dead_low:,.0f} ft AGL). Glide footprint radius
  ≈ **{r_low_m / _M_PER_NM:.2f} nm**. *Above the runway-survival altitude
  but too low to turn back* — pick a landing site within this circle.
- **Yellow dot + faint yellow circle** — engine failure at the **top** of
  the critical zone ({dead_high:,.0f} ft AGL — the turnback critical
  altitude). Glide footprint ≈ **{r_high_m / _M_PER_NM:.2f} nm**. Above
  this altitude the turnback becomes feasible.
- **Dashed blue line** — wind direction (from).
- L/D used = **{ld_ratio:.1f}** (best glide, with prop drag and current
  configuration). Wind effects are noted above but not drawn on the
  circles.

**Use the satellite imagery** to spot:
- Open fields, golf courses, dry lake beds, large parking lots → good.
- Highways → only in extremis (wires, traffic, signage).
- Water, dense forest, residential blocks → avoid.

When OSM landing candidates are enabled, color-coded overlays mark these
features automatically. Toggle layers via the control in the upper-right
of the map.
""")
