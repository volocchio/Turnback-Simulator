"""
Thrust Deck Module

This module contains thrust data tables for aircraft engines.
- FJ44-class engines: Textron data for Citation jets
- PW2040 (F117-PW-100): C-17A performance manual data (military derivative of 757 engine)

Data is organized as thrust (lbf) vs altitude (ft) and Mach number.
"""

import numpy as np
from scipy.interpolate import RectBivariateSpline

# ═══════════════════════════════════════════════════════════════════════
# FJ44 Thrust Deck — Textron data for Citation jets
# ═══════════════════════════════════════════════════════════════════════

THRUST_DECK_ALTITUDES = np.array([
    0, 5000, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000, 50000
])

THRUST_DECK_MACH = np.array([
    0.01, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 
    0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95
])

# Thrust data [altitude, mach] in lbf per engine
THRUST_DECK_DATA = np.array([
    # Mach: 0.01   0.10   0.15   0.20   0.25   0.30   0.35   0.40   0.45   0.50   0.55   0.60   0.65   0.70   0.75   0.80   0.85   0.90   0.95
    [2820, 2590, 2495, 2400, 2300, 2200, 2125, 2050, 1965, 1880, 1850, 1820, 1820, 1820, 1820, 1820, 1820, 1820, 1820],  # 0 ft
    [2550, 2358, 2279, 2200, 2118, 2036, 1968, 1900, 1845, 1790, 1773, 1755, 1748, 1740, 1740, 1740, 1735, 1730, 1730],  # 5000 ft
    [2280, 2125, 2063, 2000, 1936, 1872, 1811, 1750, 1725, 1700, 1695, 1690, 1675, 1660, 1660, 1660, 1650, 1640, 1640],  # 10000 ft
    [1980, 1895, 1848, 1800, 1741, 1681, 1633, 1585, 1561, 1538, 1524, 1510, 1495, 1480, 1474, 1468, 1455, 1443, 1443],  # 15000 ft
    [1680, 1665, 1633, 1600, 1545, 1490, 1455, 1420, 1398, 1375, 1353, 1330, 1315, 1300, 1288, 1275, 1261, 1247, 1247],  # 20000 ft
    [1390, 1378, 1358, 1338, 1303, 1268, 1246, 1225, 1213, 1200, 1195, 1190, 1184, 1179, 1172, 1166, 1160, 1154, 1154],  # 25000 ft
    [1100, 1090, 1083, 1075, 1060, 1045, 1038, 1030, 1028, 1025, 1038, 1050, 1054, 1057, 1057, 1057, 1059, 1062, 1062],  # 30000 ft
    [880,  880,  880,  880,  880,  880,  880,  880,  885,  890,  895,  900,  908,  915,  920,  925,  932,  938,  938],   # 35000 ft
    [680,  680,  680,  680,  680,  680,  680,  680,  680,  680,  690,  700,  710,  720,  728,  735,  744,  753,  753],   # 40000 ft
    [510,  510,  510,  510,  510,  510,  510,  510,  510,  510,  510,  510,  519,  527,  531,  535,  542,  549,  549],   # 45000 ft
    [300,  305,  309,  313,  320,  328,  331,  335,  336,  338,  336,  335,  347,  359,  366,  374,  384,  395,  395],   # 50000 ft
])


class ThrustDeck:
    """
    Thrust deck interpolator.
    
    Uses bivariate spline interpolation for smooth thrust values
    across altitude and Mach number. Supports custom altitude/Mach/data
    arrays for different engine types.
    """
    
    def __init__(self, thrust_multiplier=1.0, altitudes=None, machs=None, data=None):
        """
        Initialize thrust deck interpolator.
        
        Args:
            thrust_multiplier: Multiplier to scale thrust (e.g., for different engine variants)
            altitudes: Custom altitude array (default: FJ44 altitudes)
            machs: Custom Mach array (default: FJ44 Mach numbers)
            data: Custom thrust data array [alt, mach] (default: FJ44 data)
        """
        self.thrust_multiplier = thrust_multiplier
        self._altitudes = altitudes if altitudes is not None else THRUST_DECK_ALTITUDES
        self._machs = machs if machs is not None else THRUST_DECK_MACH
        self._data = data if data is not None else THRUST_DECK_DATA
        
        # Create interpolator (kx=3, ky=3 for cubic splines)
        self.interpolator = RectBivariateSpline(
            self._altitudes,
            self._machs,
            self._data,
            kx=3,
            ky=3
        )
    
    def get_thrust(self, altitude_ft, mach, num_engines=2):
        """
        Get thrust at specified altitude and Mach number.
        
        Args:
            altitude_ft: Pressure altitude in feet
            mach: Mach number
            num_engines: Number of engines (default: 2)
        
        Returns:
            Total thrust in lbf for all engines
        """
        # Clamp inputs to valid range
        alt = np.clip(altitude_ft, self._altitudes[0], self._altitudes[-1])
        m = np.clip(mach, self._machs[0], self._machs[-1])
        
        # Interpolate thrust per engine
        thrust_per_engine = float(self.interpolator(alt, m)[0, 0])
        
        # Apply multiplier and number of engines
        total_thrust = thrust_per_engine * self.thrust_multiplier * num_engines
        
        # Ensure minimum thrust
        return max(total_thrust, 100.0)
    
    def get_thrust_array(self, altitude_ft_array, mach_array, num_engines=2):
        """
        Vectorized version for arrays of altitude and Mach.
        
        Args:
            altitude_ft_array: Array of pressure altitudes in feet
            mach_array: Array of Mach numbers
            num_engines: Number of engines (default: 2)
        
        Returns:
            Array of total thrust in lbf for all engines
        """
        # Ensure inputs are numpy arrays
        alt = np.atleast_1d(altitude_ft_array)
        m = np.atleast_1d(mach_array)
        
        # Clamp to valid range
        alt = np.clip(alt, self._altitudes[0], self._altitudes[-1])
        m = np.clip(m, self._machs[0], self._machs[-1])
        
        # Interpolate
        thrust_per_engine = self.interpolator(alt, m, grid=False)
        
        # Apply multiplier and number of engines
        total_thrust = thrust_per_engine * self.thrust_multiplier * num_engines
        
        # Ensure minimum thrust
        return np.maximum(total_thrust, 100.0)


# Pre-instantiated thrust decks for common engine variants
# Thrust multipliers based on aircraft_config.py thrust_mult values
THRUST_DECKS = {
    'FJ44-1A': ThrustDeck(thrust_multiplier=0.674),   # CJ, CJ1
    'FJ44-1AP': ThrustDeck(thrust_multiplier=0.697),  # CJ1+
    'FJ44-2A': ThrustDeck(thrust_multiplier=0.723),   # M2
    'FJ44-3A': ThrustDeck(thrust_multiplier=0.851),   # CJ2
    'FJ44-3AP': ThrustDeck(thrust_multiplier=0.883),  # CJ2+
    'FJ44-4A': ThrustDeck(thrust_multiplier=1.0),     # CJ3, CJ3+
    'FJ44-4AE': ThrustDeck(thrust_multiplier=1.284),  # CJ4
    'PW615': ThrustDeck(thrust_multiplier=0.518),     # Mustang (PW615F-A, 1460 lb/ea)
}


# ═══════════════════════════════════════════════════════════════════════
# PW2040 / F117-PW-100 Thrust Deck — C-17A performance manual data
# Military derivative of the PW2037 (Boeing 757), BPR ≈ 6:1
# Thrust per engine in lbf — scaled to published TO-rated level.
#
# Published F117-PW-100 static takeoff thrust: 40,440 lb/engine.
# Original MCT baseline SL/static: 36,500 lb → scale = 40440/36500 ≈ 1.108.
# The deck is stored at the published TO level so that the sim's
# TAKEOFF_THRUST_BOOST (×1.07) is NOT applied on top for C-17.
# Cruise thrust is governed by throttle_factor < 1.0.
# ═══════════════════════════════════════════════════════════════════════
_PW2040_SCALE = 40440.0 / 36500.0  # 1.1079…  MCT→TO calibration

PW2040_ALTITUDES = np.array([
    0, 5000, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000
])

PW2040_MACH = np.array([
    0.00, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.74, 0.80, 0.90
])

# Thrust data [altitude_index, mach_index] in lbf per engine (scaled to TO-rated)
PW2040_DATA = (np.array([
    # Mach:  0.00   0.10   0.20   0.30   0.40   0.50   0.60   0.70   0.74   0.80   0.90
    [36500, 36200, 35900, 35650, 35300, 34750, 34400, 34100, 34000, 33600, 33000],  # 0 ft
    [35500, 35300, 35100, 34850, 34650, 34100, 33800, 33500, 33400, 33100, 32500],  # 5000 ft
    [34300, 34100, 33950, 33750, 33600, 33300, 32900, 32700, 32600, 32300, 31700],  # 10000 ft
    [32800, 32600, 32500, 32300, 32150, 32000, 31700, 31800, 31800, 31600, 30900],  # 15000 ft
    [30500, 30400, 30300, 30250, 30150, 30050, 29950, 29900, 29800, 29500, 28800],  # 20000 ft
    [27500, 27450, 27400, 27350, 27300, 27250, 27200, 26950, 26850, 26600, 25800],  # 25000 ft
    [24000, 23950, 23900, 23900, 23900, 23850, 23800, 23600, 23500, 23200, 22400],  # 30000 ft
    [20500, 20850, 21200, 21600, 22000, 22300, 22700, 22500, 22400, 22100, 21300],  # 35000 ft
    [17200, 17550, 17900, 18250, 18600, 18950, 19300, 19100, 19000, 18700, 17900],  # 40000 ft
    [14500, 14750, 15050, 15300, 15600, 15850, 16100, 15900, 15800, 15500, 14700],  # 45000 ft
], dtype=float) * _PW2040_SCALE).astype(int)

# Pre-instantiated PW2040 deck (TO-rated thrust values)
PW2040_DECK = ThrustDeck(
    thrust_multiplier=1.0,
    altitudes=PW2040_ALTITUDES,
    machs=PW2040_MACH,
    data=PW2040_DATA,
)

# Registry of native engine decks (aircraft uses this deck directly, thrust_mult = 1.0)
NATIVE_ENGINE_DECKS = {
    'F117-PW-100': PW2040_DECK,
    'PW2040': PW2040_DECK,
}

# Aircraft models that use a native engine deck (bypass FJ44 + multiplier)
_NATIVE_DECK_AIRCRAFT = {
    'C-17': 'F117-PW-100',
    'C-17 ER': 'F117-PW-100',
}


# Cache for custom-multiplier ThrustDeck instances to avoid rebuilding RectBivariateSpline every call
_thrust_deck_cache: dict = {}


def get_thrust_for_aircraft(aircraft_model, altitude_ft, mach, num_engines=2, thrust_multiplier=None):
    """
    Convenience function to get thrust for a specific aircraft model.
    
    Args:
        aircraft_model: Aircraft model string (e.g., 'CJ1', 'M2', 'CJ3', 'C-17')
        altitude_ft: Pressure altitude in feet
        mach: Mach number
        num_engines: Number of engines (default: 2)
        thrust_multiplier: Optional override for thrust multiplier (ignored for native-deck aircraft)
    
    Returns:
        Total thrust in lbf
    """
    # Check for native engine deck first (C-17, etc.)
    native_engine = _NATIVE_DECK_AIRCRAFT.get(aircraft_model)
    if native_engine is not None:
        deck = NATIVE_ENGINE_DECKS[native_engine]
        return deck.get_thrust(altitude_ft, mach, num_engines)

    # Map aircraft models to FJ44 engine types
    engine_map = {
        'CJ': 'FJ44-1A',
        'CJ1': 'FJ44-1A',
        'CJ1+': 'FJ44-1AP',
        'M2': 'FJ44-2A',
        'CJ2': 'FJ44-3A',
        'CJ2+': 'FJ44-3AP',
        'CJ3': 'FJ44-4A',
        'CJ3+': 'FJ44-4A',
        'CJ4': 'FJ44-4AE',
        'Mustang': 'PW615',
    }
    
    engine_type = engine_map.get(aircraft_model, 'FJ44-1A')
    
    if thrust_multiplier is not None:
        # Use cached deck to avoid rebuilding RectBivariateSpline on every call
        if thrust_multiplier not in _thrust_deck_cache:
            _thrust_deck_cache[thrust_multiplier] = ThrustDeck(thrust_multiplier=thrust_multiplier)
        deck = _thrust_deck_cache[thrust_multiplier]
        return deck.get_thrust(altitude_ft, mach, num_engines)
    else:
        # Use pre-instantiated deck
        deck = THRUST_DECKS[engine_type]
        return deck.get_thrust(altitude_ft, mach, num_engines)


if __name__ == "__main__":
    # Test the thrust deck
    print("Testing Thrust Deck Interpolation")
    print("=" * 60)
    
    # Test at sea level
    print("\nSea Level (0 ft):")
    for m in [0.1, 0.3, 0.5, 0.7]:
        thrust = get_thrust_for_aircraft('CJ1', 0, m, num_engines=2)
        print(f"  Mach {m:.2f}: {thrust:.0f} lbf (2 engines)")
    
    # Test at cruise altitude
    print("\nCruise (41,000 ft):")
    for m in [0.5, 0.6, 0.7]:
        thrust = get_thrust_for_aircraft('CJ1', 41000, m, num_engines=2)
        print(f"  Mach {m:.2f}: {thrust:.0f} lbf (2 engines)")
    
    # Compare different aircraft
    print("\nAircraft Comparison at 35,000 ft, Mach 0.70:")
    for model in ['CJ1', 'M2', 'CJ3', 'CJ4']:
        thrust = get_thrust_for_aircraft(model, 35000, 0.70, num_engines=2)
        print(f"  {model}: {thrust:.0f} lbf")
