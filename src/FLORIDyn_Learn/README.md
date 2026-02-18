# FLORIDyn_Learn

A working sandbox for running and controlling **FLORIDyn** (OFF — Open-source Farm Dynamics) simulations. FLORIDyn is the dynamic counterpart of FLORIS: instead of a steady-state wake snapshot, it propagates wakes in time using moving observation points, making it suitable for dynamic control experiments.

---

## Project Structure

```
FLORIDyn_Learn/
├── main.py                  ← Entry point. Write your control logic here.
├── Inputs/
│   ├── run_2T_NREL5MW.yaml  ← Simulation config (2 × NREL 5 MW, 120 s)
│   ├── run_2T.yaml          ← Alternative config (2 × IEA 10 MW)
│   └── FLORIS/              ← FLORIS wake model config files (don't edit)
└── off/                     ← FLORIDyn framework (don't edit unless needed)
    ├── off.py               ← Simulation engine (run_sim loop)
    ├── off_interface.py     ← High-level API
    ├── controller.py        ← Controller classes including DynamicYawController
    ├── turbine.py           ← Turbine model
    ├── windfarm.py          ← Wind farm container
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

The simulation runs for 120 s (30 × 4 s timesteps), prints per-turbine power statistics, shows a power time-series plot, and saves results to `../runs/`.

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

**Time-scheduled step change (apply steering after 60 s):**
```python
def my_control(turbines, t):
    if t < 60.0:
        return [0.0, 0.0]
    else:
        return [20.0, 0.0]
```

**Reactive control (read wind direction and respond):**
```python
def my_control(turbines, t):
    yaw_commands = []
    for i, turbine in enumerate(turbines):
        wind_dir = turbine.ambient_states.get_wind_dir_ind(0)
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

The physical yaw rate limit is **0.3 deg/s** (NREL 5 MW spec), set in the turbine config. A 20° change takes ~67 s at full rate. In the current `DynamicYawController` the yaw rate limit is **not enforced** — commands are applied instantaneously. Add rate limiting in `controller.py` if needed.

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
| `control_applied` | `yaw` | Applied yaw misalignment [deg] |
| `control_applied` | `commanded_orientation` | Commanded nacelle orientation [deg] |

Results are also saved as CSV files to `../runs/<run_id>/`.
