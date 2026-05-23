# drone_sim — Godot 4 GDExtension Drone Simulator

Open-source, Windows/Linux/macOS.  
Physics computed in C++20 via Godot's `_integrate_forces` hook — zero GDScript in the hot path.

---

## Architecture

```
DroneBody (RigidBody3D subclass)
├── RotorArray
│   └── BladeElementSolver × N   ← BET thrust/torque per rotor
├── Atmosphere
│   ├── ISA model                ← density/pressure/temperature vs altitude
│   └── Dryden turbulence        ← MIL-HDBK-1797 wind model
├── AeroEffectsBundle
│   ├── GroundEffectModel        ← Cheeseman-Bennett IGE
│   └── VortexRingStateModel     ← Leishman VRS onset + thrust degradation
├── FlightController             ← PX4-style cascade PID (attitude + rate)
├── MixerMatrix                  ← wrench → per-rotor throttle allocation
└── SensorSuite
    ├── IMU                      ← accel/gyro noise, bias drift
    ├── Barometer                ← pressure altitude, lag
    └── GPS                      ← position/velocity, 10 Hz update
```

---

## Prerequisites

| Tool | Version |
|------|---------|
| CMake | ≥ 3.22 |
| C++ compiler | MSVC 2022 / GCC 12+ / Clang 15+ |
| Godot | 4.2 or later |
| Python | 3.8+ (for godot-cpp SConstruct, if needed) |

---

## Build (Windows — MSVC)

```bat
git clone --recurse-submodules https://github.com/YOUR_ORG/drone_sim.git
cd drone_sim

:: Fetch godot-cpp (Godot 4.2 branch)
git submodule update --init --recursive

:: Configure
cmake -B build -G "Visual Studio 17 2022" -A x64 ^
      -DCMAKE_BUILD_TYPE=Release

:: Build
cmake --build build --config Release --parallel

:: Output: gdextension/bin/drone_sim.windows.release.x86_64.dll
```

## Build (Linux / macOS)

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
# Output: gdextension/bin/libdrone_sim.linux.release.x86_64.so
```

## Optional flags

```bash
-DDRONE_SIM_ASAN=ON       # AddressSanitizer
-DDRONE_SIM_PROFILE=ON    # gprof / perf friendly
-DDRONE_SIM_TESTS=ON      # build unit tests (requires Catch2)
```

---

## Integrating into a Godot 4 project

1. Copy `gdextension/` folder into your Godot project root.
2. In Godot, open *Project → Project Settings → Plugins* and enable **drone_sim**.
3. Add a `DroneBody` node to your scene (it appears under RigidBody3D).
4. Attach `demo/scripts/drone_controller.gd` to a sibling Node and set the `drone` export.

### Scene minimum requirements

```
World
├── DroneBody          ← C++ physics node
│   ├── CollisionShape3D (ConvexPolygonShape3D or CapsuleShape3D)
│   └── MeshInstance3D (visual mesh)
├── Node               ← drone_controller.gd attached here
├── Camera3D
└── DirectionalLight3D
```

---

## Aerodynamics layers

### Blade Element Theory (BET)

Each rotor is divided into `N_ANNULI = 24` radial sections.  
Per annulus:
- Local AoA = blade pitch θ(r) − inflow angle φ(r)  
- dT = ½ρ(Ωr)² c Cl dr  
- dQ = ½ρ(Ωr)² c Cd r dr  

Induced velocity `vi` is resolved with 3 Newton iterations of Rankine-Froude momentum theory per tick.  
Motor dynamics: 1st-order ESC lag (`esc_tau = 15 ms`), gyroscopic precession included.

### Ground Effect

Cheeseman-Bennett (1955):
```
T_IGE / T_OGE = 1 / (1 − (R/4h)²)
```
Blends out smoothly above `h/R = 3`.

### Vortex Ring State

Leishman (2000) onset envelope:
- Descent rate: `0.3 Vc ≤ Vd ≤ 1.5 Vc`
- Lateral speed: `< 0.5 Vc`

Hysteresis filter: 0.5 s build-up, 1.2 s recovery.  
Thrust degradation: up to 30% with low-frequency buffeting at ~2.3 Hz.

### ISA Atmosphere

Standard troposphere with configurable sea-level altitude offset.  
Sutherland's law for dynamic viscosity.

### Dryden Turbulence

1st-order shaping filter per axis. Scale length and intensity configurable at runtime.

---

## Tuning the PID

Call from GDScript at runtime:

```gdscript
drone.set_rate_roll_pid(0.15, 0.05, 0.003)
drone.set_rate_pitch_pid(0.15, 0.05, 0.003)
drone.set_rate_yaw_pid(0.20, 0.10, 0.0)
```

Or wire to a debug UI panel for live tuning.

---

## Telemetry Dictionary

`drone.get_telemetry()` returns every physics tick:

| Key | Type | Unit |
|-----|------|------|
| `altitude` | float | m |
| `ground_speed` | float | m/s |
| `vertical_speed` | float | m/s |
| `roll_deg` / `pitch_deg` / `yaw_deg` | float | ° |
| `roll_rate` / `pitch_rate` / `yaw_rate` | float | rad/s |
| `total_thrust` | float | N |
| `power_draw` | float | W |
| `vrs_active` | bool | — |
| `vrs_severity` | float | 0–1 |
| `ground_effect_factor` | float | ≥1.0 |
| `air_density` | float | kg/m³ |
| `wind` | Vector3 | m/s |

---

## Extending

- **Add a rotor configuration**: create a new `RotorConfig`, push to `RotorArray`, add a row to `MixerMatrix`.
- **Custom mixer**: subclass `MixerMatrix` or provide a new `std::vector<Row>`.
- **Swappable FC**: replace `FlightController` with any class exposing the same `update()` signature.
- **Battery model**: replace the placeholder in `_integrate_forces` — track `Σpower_draw × dt / capacity_Wh`.

---

## License

MIT
