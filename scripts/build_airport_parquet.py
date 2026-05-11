"""Build parquet copies of airports_full.csv / runways.csv for fast load.

Run this once whenever the source CSVs change.  The Streamlit app prefers
the .parquet copies and falls back to CSV if missing.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from engine.airport_db import load_airports, _load_runways_raw, _DATA_DIR


def main() -> None:
    airports = load_airports()
    out_a = _DATA_DIR / "airports_full.parquet"
    airports.to_parquet(out_a, index=False, compression="zstd")
    print(f"wrote {out_a}  ({out_a.stat().st_size/1e6:.2f} MB, {len(airports):,} rows)")

    runways = _load_runways_raw()
    out_r = _DATA_DIR / "runways.parquet"
    runways.to_parquet(out_r, index=False, compression="zstd")
    print(f"wrote {out_r}  ({out_r.stat().st_size/1e6:.2f} MB, {len(runways):,} rows)")


if __name__ == "__main__":
    main()
