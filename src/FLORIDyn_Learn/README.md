# FLORIDyn_Learn

A working sandbox for running and controlling **FLORIDyn** (OFF — Open-source Farm Dynamics) simulations. FLORIDyn is the dynamic counterpart of FLORIS: instead of a steady-state wake snapshot, it propagates wakes in time using moving observation points, making it suitable for dynamic control experiments.

---

## Project Structure

```
FLORIDyn_Learn/
├── main.py                  ← Entry point. Write your control logic here.
├── Inputs/
│   ├── run_2T_NREL5MW.yaml  ← Simulation config (2 × NREL 5 MW, 120 s)  [default]
│   ├── run_2T.yaml          ← Alternative config (2 × IEA 10 MW)
│   └── FLORIS/              ← FLORIS wake model config files (don't edit)
└── off/                     ← FLORIDyn framework (don't edit unless needed)
    ├── off.py               ← Simulation engine (run_sim loop)
    ├── off_interface.py     ← High-level API (OFFInterface)
    ├── controller.py        ← Controller classes (see below)
    ├── turbine.py           ← Turbine model (HAWT_ADM)
    ├── windfarm.py          ← Wind farm container
    ├── ambient.py           ← Ambient state models
    ├── states.py            ← Turbine state models
    ├── utils.py             ← Coordinate utilities (ot_get_yaw, ot_get_orientation)
    └── ...
```

---

## Quick Start

Activate the environment and run:

```bash
conda activate Floris
cd FLORIDyn_Learn
python main.py
```

The simulation runs for 120 s (30 × 4 s timesteps). By default, `my_control` in `main.py` holds all turbines aligned for the first 20 s, then steers T0 by 20° for the remainder. After the run it prints per-turbine power statistics, shows a power time-series plot, and saves results to `../runs/`.

---

## How the Simulation Works

Each timestep the loop runs four phases in order:

```
1. PREDICT   → wake solver computes rotor wind speed → power calculated
2. CORRECT   → ambient conditions updated
3. PROPAGATE → observation points and states shift forward in time
4. CONTROL   → your control function is called → yaw commands applied
```

The key implication: **power at time `t` is calculated from the yaw set at time `t-dt`** (previous timestep). The control command you issue at time `t` takes effect from time `t+dt` onward.

---

## Writing Control Logic

Everything you need to touch is in `main.py`. Define `my_control` and it gets called once per timestep.

### Function signature

```python
def my_control(turbines, t):
    """
    turbines : list of Turbine objects (one per wind turbine)
    t        : current simulation time [s]

    Returns  : list[float] — desired yaw misalignment [deg] per turbine
               positive = nacelle rotates clockwise away from wind
    """
    ...
    return yaw_commands   # e.g. [20.0, 0.0] for 2 turbines
```

Define it above `main()` and attach it before running:

```python
oi.init_simulation_by_path(config_path)
oi.off_sim.controller.set_control_fn(my_control)
oi.run_sim()
```

### Useful turbine state you can read inside `my_control`

| Expression | Returns |
|---|---|
| `turbine.ambient_states.get_wind_dir_ind(0)` | Wind direction at turbine [deg] |
| `turbine.ambient_states.get_turbine_wind_speed_abs()` | Effective rotor wind speed [m/s] |
| `turbine.get_yaw_orientation()` | Current nacelle orientation [deg] |
| `turbine.calc_yaw(wind_dir)` | Current yaw misalignment [deg] |

### Examples

**Constant yaw steering (T0 steered 20°, T1 aligned):**
```python
def my_control(turbines, t):
    return [20.0, 0.0]
```

**Time-scheduled step change (hold aligned for 20 s, then steer T0):**
```python
def my_control(turbines, t):
    if t < 20.0:
        return [0.0, 0.0]
    else:
        return [20.0, 0.0]
```

**Reactive control (read wind speed and respond):**
```python
def my_control(turbines, t):
    yaw_commands = []
    for i, turbine in enumerate(turbines):
        wind_speed = turbine.ambient_states.get_turbine_wind_speed_abs()
        if i == 0 and wind_speed > 10.0:
            yaw_commands.append(20.0)
        else:
            yaw_commands.append(0.0)
    return yaw_commands
```

**No control (all turbines aligned with wind):**
```python
def my_control(turbines, t):
    return [0.0] * len(turbines)
```

---

## Simulation Configuration (`run_2T_NREL5MW.yaml`)

Key parameters you may want to change:

```yaml
sim:
  sim:
    time step: 4.0       # [s] — control is applied once per step
    time start: 0.0
    time end:   120.0    # [s] — increase for longer runs

ambient:
  flow_field:
    wind_speeds:      [12.0]   # [m/s]
    wind_directions:  [270.0]  # [deg] — 270 = wind from west (→ east)
    turbulence_intensities: [0.06]

controller:
  settings:
    ctl: "dynamic yaw controller"
    initial_orientation_deg: 270.0   # starting nacelle orientation [deg]
```

Wind farm layout (2 turbines at 5 rotor-diameters apart, inline with wind):

```yaml
wind_farm:
  farm:
    layout_x: [0.0, 5.0]   # x positions in units of D (rotor diameter)
    layout_y: [0.0, 0.0]
    unit: [D]
    diameter: [125.88]      # NREL 5 MW rotor diameter [m]
```

---

## Understanding Yaw vs. Orientation

FLORIDyn uses **orientation** (absolute nacelle heading in degrees) internally, but `my_control` works in **yaw misalignment** (degrees away from wind) for convenience.

```
orientation [deg]  = wind_direction - yaw_misalignment
yaw_misalignment   = wind_direction - orientation

Example (wind from 270°):
  yaw = 0°   → orientation = 270°  (aligned, maximum power)
  yaw = 20°  → orientation = 250°  (steered 20° clockwise, wake deflected)
```

The conversion happens inside `DynamicYawController.update()` at `controller.py:500`:
```python
self.orientation_commands[i] = wind_dir - yaw_commands[i]
```

The physical yaw rate limit is **0.3 deg/s** (NREL 5 MW spec), set in the turbine config. A 20° change takes ~67 s at full rate. In the current `DynamicYawController` the yaw rate limit is **not enforced** — commands are applied instantaneously. Add rate limiting in `controller.py` if needed.

---

## Available Controllers

All controllers live in `off/controller.py` and inherit from the abstract `Controller` base class.

| Class | Description |
|---|---|
| `DynamicYawController` | **Used by default.** Calls your `set_control_fn()` callback each timestep. Falls back to zero yaw misalignment if no callback is set. |
| `IdealGreedyBaselineController` | Always aligns every turbine instantly with the local wind direction (zero yaw misalignment, no rate limit). |
| `RealisticGreedyBaselineController` | Greedy alignment with a misalignment threshold and yaw rate limiting. |
| `YawSteeringLUTController` | Reads optimised yaw angles from a CSV look-up table keyed by wind direction. |
| `YawSteeringPrescribedMotionController` | Replays a pre-computed orientation trajectory (from CSV or YAML). |
| `DeadbandYawSteeringLuTController` | LUT-based yaw steering with a deadband + integrator trigger and yaw rate limiting. |

To switch controllers, change `ctl` in the YAML config and pass the matching settings dict. Only `DynamicYawController` is wired up in `off_interface.py` for the `"dynamic yaw controller"` key.

---

## Reading Results

After `oi.run_sim()`:

```python
measurements    = oi.measurements      # power, wind speed per turbine per timestep
control_applied = oi.control_applied   # yaw, orientation per turbine per timestep
```

Both are pandas DataFrames. Key columns:

| DataFrame | Column | Description |
|---|---|---|
| `measurements` | `t_idx` | Turbine index (0-based) |
| `measurements` | `time` | Simulation time [s] |
| `measurements` | `power_OFF` | Power output [W] |
| `control_applied` | `t_idx` | Turbine index (0-based) |
| `control_applied` | `time` | Simulation time [s] |
| `control_applied` | `yaw` | Applied yaw misalignment [deg] |
| `control_applied` | `orientation` | Actual nacelle orientation [deg] |
| `control_applied` | `commanded_orientation` | Commanded nacelle orientation [deg] |

Results can be saved with:

```python
oi.store_measurements()       # → <run_dir>/measurements.csv
oi.store_applied_control()    # → <run_dir>/applied_control.csv
oi.store_run_file()           # → <run_dir>/<config>.yaml  (copy of input)
```

Results are saved to `../runs/<run_id>/` (i.e. `src/runs/` relative to the repo root).
