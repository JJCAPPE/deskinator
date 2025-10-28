# Deskinator SLAM & Coverage — Raspberry Pi 4B Software Plan (AI Agent Spec)

This document specifies an end‑to‑end, implementation‑ready plan for an AI agent running on a Raspberry Pi 4B to perform table‑top SLAM, rectangle inference, and coverage (sweeping) for the **Deskinator** robot. All I/O pins and geometry are parameterized; I²C addresses remain configurable. The plan assumes stepper actuation with TMC2209 drivers and a front vacuum with four down‑facing APDS9960 proximity sensors behind a TCA9548A I²C mux. A BNO085 IMU provides yaw/gyro. The pose‑graph backend is intentionally lightweight (SciPy least‑squares + NetworkX).

---

## 0) Repository Layout

```
deskinator/
  README.md
  pyproject.toml
  deskinator/
    __init__.py
    config.py
    main.py
    hw/
      gpio.py
      stepper.py
      vacuum.py
      i2c.py
      apds9960.py
      tca9548a.py
      bno085.py
    slam/
      ekf.py
      posegraph.py
      rect_fit.py
      frames.py
    planning/
      coverage.py
      map2d.py
    control/
      fsm.py
      motion.py
      limits.py
    utils/
      logs.py
      viz.py
      timing.py
```

---

## 1) Configuration (central parameters)

```python
# deskinator/config.py
from dataclasses import dataclass

@dataclass
class Pins:
    RIGHT_STEP: int = 22   # Driver Right Step GPIO22
    RIGHT_DIR:  int = 23   # Driver Right Dir  GPIO23
    LEFT_DIR:   int = 21   # Driver Left  Dir  GPIO21
    LEFT_STEP:  int = 20   # Driver Left  Step GPIO20
    VACUUM_PWM:int = 5     # Vacuum MOSFET     GPIO5
    START_IN:   int | None = None  # Optional start/gesture input

@dataclass
class I2CParams:
    BUS: int = 1
    ADDR_IMU: int | None = None      # set via i2cdetect
    ADDR_MUX: int = 0x70             # TCA9548A default
    APDS_ADDR: int = 0x39            # APDS9960 default
    MUX_CHANS: tuple[int,int,int,int] = (0,1,2,3)
    LEFT_PAIR:  tuple[int,int] = (0,1)  # define exact channels later
    RIGHT_PAIR: tuple[int,int] = (2,3)

@dataclass
class Geometry:
    # Robot frame: +x forward, +y left, origin at axle midpoint.
    WHEEL_BASE: float = 0.170        # m (update from CAD)
    STEPS_PER_M: float = 6400.0      # set from microstepping/drive
    SENSOR_FWD: float = 0.080        # m; sensors lead axle
    SENSOR_LAT: tuple[float,...] = (+0.040, +0.015, -0.015, -0.040)  # m
    VAC_WIDTH:  float = 0.160        # m; effective cleaned width
    SENSOR_TO_VAC: float = 0.015     # m; vacuum ahead of sensors

@dataclass
class Limits:
    # For NEMA17 + TMC2209 @ 12V, 1A — desk-safe
    V_MAX:       float = 0.18    # m/s
    OMEGA_MAX:   float = 1.8     # rad/s
    A_MAX:       float = 0.60    # m/s^2
    ALPHA_MAX:   float = 4.0     # rad/s^2
    J_MAX:       float = 4.0     # m/s^3
    V_REV_MAX:   float = 0.06    # m/s (reverse is minimal)

@dataclass
class Algo:
    FUSE_HZ: int = 50            # EKF update rate
    EDGE_THRESH: float = 0.5     # APDS off-table threshold (norm.)
    EDGE_DEBOUNCE: float = 0.06  # s
    NODE_SPACING: float = 0.05   # m
    LOOP_RADIUS: float = 0.06    # m
    SWEEP_OVERLAP: float = 0.02  # m
    GRID_RES: float = 0.02       # m (2 cm raster)
    RECT_CONF_MIN_LEN: float = 0.6      # m of perimeter observed
    RECT_ORTHTOL_DEG: float = 8.0       # deg
    STOP_DELAY_AFTER_EDGE: float = 0.00 # s
    POST_EDGE_BACKOFF: float = 0.03     # m
    POST_EDGE_SIDE_STEP: float = 0.05   # m

PINS = Pins(); I2C = I2CParams(); GEOM = Geometry(); LIMS = Limits(); ALG = Algo()
```

**Notes**

- Keep I²C addresses as parameters until probed.
- `STEPS_PER_M` derived from step angle × microstep × mechanics.

---

## 2) Hardware Abstraction Layer (HAL)

### 2.1 StepperDrive (TMC2209 STEP/DIR)

- Converts `(v, ω)` to per-wheel velocities, limits accel/jerk, generates STEP pulses and sets DIR lines.
- Returns odometry as `(dSL, dSR)` in meters per control tick.

**API**

```python
class StepperDrive:
    def command(self, v: float, omega: float) -> None: ...
    def update(self, dt: float) -> tuple[float,float]: ...
    def read_odometry(self) -> tuple[float,float]: ...
```

**Details**

- `vL = v - 0.5*ω*WHEEL_BASE`, `vR = v + 0.5*ω*WHEEL_BASE`
- `steps/s = v * STEPS_PER_M`, trapezoidal profile obeying `LIMS`.

### 2.2 Vacuum MOSFET

```python
class Vacuum:
    def on(self, duty: float = 1.0): ...
    def off(self): ...
```

### 2.3 I²C Devices

- `tca9548a.py`: `select(channel)`
- `apds9960.py`: `init()`, `read_proximity_norm()`; provides calibrated 0..1 output, with basic filtering.
- `bno085.py`: `read_yaw_rate()`, optional `read_yaw_abs()`.

---

## 3) State Estimation

### 3.1 Kinematics

From `(dSL, dSR)`:

```
ds  = 0.5*(dSR + dSL)
dth = (dSR - dSL)/WHEEL_BASE
x += ds*cos(θ + dth/2), y += ds*sin(θ + dth/2), θ = wrap(θ + dth)
```

### 3.2 EKF on SE(2)

- State `[x, y, θ, b_g]` (optional gyro bias).
- Prediction: integrate odom with process noise for slip.
- Measurement: gyro yaw rate; optional absolute yaw (low weight).

**API**

```python
class EKF:
    def predict(self, dSL, dSR, dt): ...
    def update_gyro(self, omega_z, dt): ...
    def update_yaw_abs(self, yaw): ...
    def pose(self): -> (x,y,θ)
    def cov(self) -> np.ndarray
    def set_pose(self, x,y,θ): ...
```

### 3.3 Edge Detection Rule

- Four down-facing APDS readings → normalized `S[i]`.
- “Off-table” when `S[i] <= EDGE_THRESH` held for `EDGE_DEBOUNCE` s.
- **Braking condition (your rule):** if both sensors in **LEFT_PAIR** trip or both in **RIGHT_PAIR** trip while moving forward ⇒ **EDGE_EVENT(side='left'|'right')**.
- Only consider forward motion; reverse is control-managed only to clear the edge.

### 3.4 Edge Hit Localization

- Sensor i pose in robot frame: `(GEOM.SENSOR_FWD, GEOM.SENSOR_LAT[i])`.
- Project a small `ε` forward, transform with current `(x,y,θ)` to world, push into `edge_hits_world`.

---

## 4) Rectangle/Boundary Inference & Pose Graph

### 4.1 Rectangle Estimator (`rect_fit.py`)

- Buffer recent edge points, run RANSAC line extraction.
- Maintain four dominant directions; enforce near-orthogonality within `ALG.RECT_ORTHTOL_DEG`.
- Confirm rectangle when collected boundary length > `ALG.RECT_CONF_MIN_LEN` and four sides stabilize.
- Output: rectangle pose (center, heading), width/height, corners.

### 4.2 Pose Graph (`posegraph.py`)

- Nodes: SE(2) every `ALG.NODE_SPACING` meters.
- Edges: odometry (between neighbors), yaw (soft), optional loop-closures if re-visited within `ALG.LOOP_RADIUS` and local line directions agree.
- Optimizer: `scipy.optimize.least_squares` (LM) over compact SE(2) parameterization; book-keeping with NetworkX.

**API**

```python
class PoseGraph:
    def add_node(self, t, pose): ...
    def add_edge_odom(self, i, j, z_ij, Info): ...
    def add_edge_yaw(self, i, yaw, Info): ...
    def maybe_add_loop(self, i, candidates, rect_ctx): ...
    def optimize(self): ...
    def current_pose(self) -> (x,y,θ): ...
```

---

## 5) Coverage Mapping & Planning

### 5.1 Swept Map (`map2d.py`)

- Raster at `ALG.GRID_RES`.
- When `v > 0`, paint a rectangular brush representing vacuum footprint:
  - Width: `GEOM.VAC_WIDTH`
  - Longitudinal: from `GEOM.SENSOR_FWD + GEOM.SENSOR_TO_VAC` forward of axle along the path increment.
- Reverse motion not counted.

**API**

```python
class SweptMap:
    def add_forward_sweep(self, pose, ds): ...
    def coverage_ratio(self, rect=None) -> float: ...
```

### 5.2 Lane Planner (`coverage.py`)

- Once rectangle is confident:
  - Inset rectangle by safety margin (`GEOM.SENSOR_FWD` + stopping distance).
  - Lane spacing = `GEOM.VAC_WIDTH - ALG.SWEEP_OVERLAP`.
  - Build alternating boustrophedon (lawn‑mower) paths aligned to long side.

**API**

```python
class CoveragePlanner:
    def set_rectangle(self, rect): ...
    def build_lanes(self, ALG, GEOM) -> list[list[tuple[float,float]]]: ...
```

---

## 6) Control

### 6.1 Supervisor FSM (`fsm.py`)

Phases: `WAIT_START` → `BOUNDARY_DISCOVERY` → `COVERAGE` → `DONE`.

**Edge Event Handling (two‑sensor pair trips):**

1. **Immediate brake** (override; set v,ω→0 with accel limit).
2. **Backoff** straight by `ALG.POST_EDGE_BACKOFF` (capped by `LIMS.V_REV_MAX`).
3. **Rotate away** from the tripped side (turn interior).
4. **Side‑step along boundary** by `ALG.POST_EDGE_SIDE_STEP`.
5. Resume boundary following or lane following.

### 6.2 Motion (`motion.py`)

- **Edge follower**: keep a small inside offset to border; when no edge currently sensed, advance cautiously; react to one‑sensor warnings by slowing; if pair trips, call edge routine above.
- **Path follower**: Pure‑pursuit / PD to lane waypoints; constrain curvature by `LIMS.OMEGA_MAX`. Skip already‑covered spans using `SweptMap`.

**API**

```python
class MotionController:
    def cmd_boundary(self, ekf_pose, edge_ctx) -> tuple[float,float]: ...
    def cmd_follow_path(self, ekf_pose, path, swept_map) -> tuple[float,float]: ...
    def handle_edge_event(self, side: str): ...
    def set_lanes(self, lanes): ...
    def current_lane(self): ...
    def coverage_done(self, swept, rect) -> bool: ...
```

### 6.3 Limits & Braking (`limits.py`)

- Trapezoidal/jerk‑limited profiles enforce `LIMS`.
- Predictive slowdown near edges optional; **pair‑trip rule overrides** with immediate braking.

---

## 7) Orchestration (`main.py` with asyncio)

- **Sensing loop @ 50 Hz**: read odom, update EKF, read APDS via mux, update edge context; on new pair‑trip, trigger handler and push edge hits transformed to world.
- **Mapping loop @ 20–50 Hz**: add pose‑graph nodes/edges, attempt loop closures, optimize periodically; try rectangle fit; when rectangle confident → build lanes.
- **Control loop @ 50 Hz**: command boundary or path follower; paint swept map only for forward motion; detect completion.

Pseudo‑skeleton:

```python
async def loop_sense(): ...
async def loop_map(): ...
async def loop_ctrl(): ...
await asyncio.gather(loop_sense(), loop_map(), loop_ctrl())
```

---

## 8) Init, Calibration & Safety

- **APDS calibration**: measure on‑table vs off‑edge; set `EDGE_THRESH` mid‑way; save to JSON.
- **IMU bias**: average 2–3 s yaw‑rate on startup.
- **Motor polarity test**: brief forward, verify both wheels + sensors trend.
- **Safety**: if any sensor off‑table while stationary → inhibit forward; if EKF covariance diverges → stop and re‑center with absolute yaw.

---

## 9) Dependencies (lightweight)

- `numpy`, `scipy`, `networkx`, `smbus2`, `RPi.GPIO` (or `lgpio`)
- `matplotlib` for optional live debug

`pyproject.toml` minimal excerpt:

```toml
[project]
name = "deskinator"
version = "0.1.0"
dependencies = [
  "numpy>=1.26",
  "scipy>=1.13",
  "networkx>=3.2",
  "smbus2>=0.4",
  "RPi.GPIO; platform_system=='Linux'",
]

[tool.setuptools.packages.find]
where = ["deskinator"]
```

---

## 10) Bench Tests

1. **I²C scan**: discover IMU/APDS; update `I2CParams`.
2. **Sensor trip logic**: manually create edge; verify left/right pair rule triggers.
3. **Odometry**: commanded (v=0.05 m/s, ω=0) for 2 s → ≈0.10 m by steps.
4. **Edge routine**: simulate pair trip; confirm stop → backoff → rotate → sidestep profile distances.
5. **Rectangle fit**: replay logged edge points; verify orthogonality and perimeter thresholds.
6. **Coverage**: dry‑run lanes on a fake rectangle; validate swept ratio approaches 1.0 with expected overlap.

---

## 11) Open Parameters Needed

- `STEPS_PER_M` from your microstepping/drive.
- Exact mapping of `MUX_CHANS` to physical left→right order (to set `LEFT_PAIR`/`RIGHT_PAIR` correctly).
- `ADDR_IMU` and whether the IMU is on the main bus (typical) or behind mux (uncommon).
- Any preferred default lane orientation (default auto long‑side).

---

## 12) Acceptance Criteria (Definition of Done)

- Robot builds a consistent rectangular boundary (orthogonality ±8°) or a closed loop with loop‑closures under 6 cm.
- Swept‑map coverage of the inferred rectangle ≥ 97% with overlap `ALG.SWEEP_OVERLAP`.
- Edge safety rule prevents forward overrun; reverse distances remain within `ALG.POST_EDGE_BACKOFF` (±20%).
- Pose drift < 3 cm after full boundary loop when pose‑graph optimization is enabled.
