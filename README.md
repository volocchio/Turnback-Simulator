# Turnback Simulator

Engine-failure turnback envelope simulator — visualizes the safe-return zone (heart-shaped envelope) when an engine fails after takeoff.

## Features

- **3D + 2D visualization** of the turnback envelope at various failure altitudes
- **Physics-based simulation**: zero-thrust glide, coordinated turns, load factor effects, stall warning
- **Wind effects**: wind speed and direction relative to runway heading
- **Runway model**: optional finite-length runway with landing-flap auto-deploy and forward slip
- **Optimizer**: recommends best bank angle, turn direction, and flap strategy for maximum turnback altitude
- **Aircraft library**: supports single-engine aircraft from the OEM database

## Live Demo

https://turnback.voloaltro.tech

## Local Development

### Setup

```bash
# Clone and install dependencies
git clone https://github.com/volocchio/Turnback-Simulator
cd Turnback-Simulator
pip install -r requirements.txt
```

### Run Locally

```bash
streamlit run turnback_app.py
```

Open http://localhost:8501 in your browser.

### Docker

```bash
docker compose up --build
```

The app runs on `http://127.0.0.1:8510` (or your Caddy reverse proxy).

## Deployment

### Deploy to VPS

```bash
git add .
git commit -m "Your message"
.\deploy.ps1
```

The deploy script handles:
- `git push` to GitHub
- SSH into VPS
- `git pull` in the repo directory
- `docker compose up --build` to rebuild and restart containers

## Physics & Model

### Core Equations

1. **Glide gradient** (zero thrust):
   ```
   H_dot = -(TAS * m²·g) / (L/D)
   H_dot = -D / (nz * W)  [altitude loss rate]
   ```

2. **Load factor in turn**:
   ```
   nz = 1 / cos(bank)
   CL_turn = nz * W / (q * S)
   Vs_turn = Vs_clean * √(nz)
   ```

3. **Turn radius**:
   ```
   R = V_TAS² / (g * tan(bank))
   ```

4. **Best glide speed** (max L/D):
   ```
   CL_opt = √(CDo / k)
   L/D_max = CL_opt / (2·CDo)
   V_bg = √(2·nz·W / (ρ·S·CL_opt))
   ```

5. **Atmosphere model**: ISA with arbitrary deviation, field elevation, Mach effects

### Configuration

- Aircraft library in `engine/aircraft_config.py`
- Aerodynamic coefficients (CDo, Clmax, Oswald efficiency)
- Flap and gear drag increments (ΔCDo_flap, ΔCDo_gear)
- Prop drag options (feathered, windmilling, stopped)

## References

- **POH Data**: aircraft-specific stall speeds, L/D envelope
- **Physics**: Standard glide-envelope theory + load-factor corrections
- **ISA Model**: ICAO Standard Atmosphere with user-configurable offsets

## Author

Built for Tamarack Aviation by volocchio team.

## License

Internal use only.
