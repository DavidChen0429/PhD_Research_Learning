# FFPythonConnect - FAST.Farm Python Interface

A Python interface for bidirectional communication with FAST.Farm wind farm simulator using MPI (Message Passing Interface).

## Overview

This module enables Python to:
- Spawn and control FAST.Farm simulations
- Send turbine control commands (yaw, pitch, torque)
- Receive real-time measurements (power, wind speed, loads, etc.)

```
┌─────────────────┐         MPI          ┌─────────────────┐
│                 │  ──── Commands ────► │                 │
│     Python      │                      │    FAST.Farm    │
│  (Controller)   │  ◄── Measurements ── │   (Simulator)   │
│                 │                      │                 │
└─────────────────┘                      └─────────────────┘
```

## Requirements

- Python 3.8+
- NumPy
- Pandas
- mpi4py
- FAST.Farm executable (with MPI support)

### Installation

```bash
pip install numpy pandas mpi4py
```

## Quick Start

### Basic Usage

```python
import numpy as np
from FFPythonConnect import FastFarmInterface

# Create interface
interface = FastFarmInterface(
    num_turbines=3,
    fstf_file="path/to/simulation.fstf",
    max_iterations=1000,
)

# Initialize (spawns FAST.Farm process)
interface.initialize()

# Control loop
for i in range(1000):
    # Send yaw commands (degrees)
    yaw_angles = np.array([270.0, 275.0, 270.0])
    finished = interface.send_commands(yaw=yaw_angles)

    # Get measurements
    powers = interface.get_powers()          # Per-turbine power [W]
    total = interface.get_total_farm_power() # Total farm power [W]
    wind = interface.get_wind_measurements() # [speed, direction]

    if finished:
        break
```

### Using the Factory Function

If your environment has `wfcrl.simul_utils` available:

```python
from FFPythonConnect import create_fastfarm_interface

# Automatically reads num_turbines and max_iterations from file
interface = create_fastfarm_interface("path/to/simulation.fstf")
interface.initialize()
```

## API Reference

### FastFarmInterface

#### Constructor

```python
FastFarmInterface(
    num_turbines: int,           # Number of turbines in simulation
    fstf_file: str,              # Path to .fstf input file
    max_iterations: int = 10000, # Maximum simulation steps
    executable_path: str = None, # Path to FAST.Farm exe (auto-detected)
    buffer_capacity: int = 50000,# History buffer size
    averaging_window: int = 1,   # Default averaging window
    log_file: str = None,        # Optional log file path
)
```

#### Methods

| Method | Description | Returns |
|--------|-------------|---------|
| `initialize(wind_speed, wind_direction)` | Start FAST.Farm and perform handshake | None |
| `send_commands(yaw, pitch, torque)` | Send controls, advance simulation | `bool` (True if finished) |
| `get_powers(window)` | Get averaged turbine powers | `np.ndarray` |
| `get_total_farm_power(window)` | Get sum of all turbine powers | `float` |
| `get_wind_measurements(window)` | Get [speed, direction] | `np.ndarray` |
| `get_measurement(name)` | Get specific measurement by name | `np.ndarray` |
| `get_all_measurements()` | Get all measurements as DataFrame | `pd.DataFrame` |
| `get_yaw_command()` | Get current yaw command (degrees) | `np.ndarray` or None |
| `get_pitch_command()` | Get current pitch command (degrees) | `np.ndarray` or None |
| `get_torque_command()` | Get current torque command | `np.ndarray` or None |

#### Properties

| Property | Description | Type |
|----------|-------------|------|
| `wind_speed` | Current averaged wind speed [m/s] | `float` |
| `wind_direction` | Current averaged wind direction [deg] | `float` |
| `current_iteration` | Current simulation step | `int` |
| `is_finished` | Whether max iterations reached | `bool` |
| `num_turbines` | Number of turbines | `int` |

### Available Measurements

Access via `get_measurement(name)`:

| Name | Description | Shape |
|------|-------------|-------|
| `wind_speed` | Local wind speed at each turbine | `(num_turbines,)` |
| `wind_direction` | Local wind direction at each turbine | `(num_turbines,)` |
| `power` | Power output | `(num_turbines,)` |
| `yaw` | Current yaw angle | `(num_turbines,)` |
| `pitch` | Current pitch angle | `(num_turbines,)` |
| `torque` | Current torque | `(num_turbines,)` |
| `load` | Load components (6 values per turbine) | `(num_turbines, 6)` |
| `freewind_measurements` | Upstream wind [speed, dir] | `(2,)` |

## Communication Protocol

### Message Tags

| Tag | Value | Direction | Content |
|-----|-------|-----------|---------|
| COMMUNICATION | 0 | Both | Handshake data |
| YAW | 1 | Python → FF | Yaw commands |
| PITCH | 2 | Python → FF | Pitch commands |
| TORQUE | 3 | Python → FF | Torque commands |
| MEASURES | 4 | FF → Python | All measurements |

### Command Format

Each command array has the format:
```
[active_flag, turbine_1, turbine_2, ..., turbine_N]
```

- `active_flag = 1.0`: Apply this command
- `active_flag = 0.0`: Ignore (use default behavior)
- Yaw and pitch values are in **radians** over the wire (converted automatically)

### Handshake Sequence

```
Python                              FAST.Farm
   │                                    │
   │──────── Spawn Process ────────────►│
   │                                    │
   │◄─────── num_measures ──────────────│  (How many values per turbine)
   │──────── max_iterations ───────────►│  (When to stop)
   │                                    │
   │============ Ready ================│
```

### Per-Iteration Sequence

```
Python                              FAST.Farm
   │                                    │
   │──────── yaw_command ──────────────►│
   │──────── pitch_command ────────────►│
   │──────── torque_command ───────────►│
   │                                    │
   │         (simulation step)          │
   │                                    │
   │◄─────── measurements ──────────────│
   │◄─────── barrier sync ─────────────►│
   │                                    │
```

## Examples

### Yaw Control for Wake Steering

```python
import numpy as np
from FFPythonConnect import FastFarmInterface

interface = FastFarmInterface(
    num_turbines=3,
    fstf_file="3turbine_farm.fstf",
    max_iterations=500,
    averaging_window=10,  # Smooth noisy measurements
)

interface.initialize()

# Apply wake steering: upstream turbine yawed to deflect wake
for step in range(500):
    # Upstream turbine (T1) yawed 20 deg, others aligned
    yaw = np.array([250.0, 270.0, 270.0])

    done = interface.send_commands(yaw=yaw)

    if step % 50 == 0:
        power_mw = interface.get_total_farm_power() / 1e6
        print(f"Step {step}: Farm Power = {power_mw:.2f} MW")

    if done:
        break
```

### Logging Communication

```python
interface = FastFarmInterface(
    num_turbines=3,
    fstf_file="farm.fstf",
    max_iterations=100,
    log_file="debug_log.txt",  # Enable logging
)
```

Log output format:
```
Iteration 1:
  Commands - YAW: [270. 275. 270.] | PITCH: None | TORQUE: None
  Raw Power: [2.5e6 2.3e6 2.4e6]
  Avg Power (window=1): [2.5e6 2.3e6 2.4e6]
  Wind: speed=8.50 m/s, dir=270.0 deg
---
```

### Accessing All Measurements

```python
# Get as DataFrame
df = interface.get_all_measurements()
print(df)

#    wind_speed    power  wind_direction    yaw  pitch  torque  load_0  ...
# 0        8.5  2.5e+06           270.0  270.0    0.0     0.0   1e+07  ...
# 1        7.2  2.3e+06           270.0  275.0    0.0     0.0   1e+07  ...
# 2        6.8  2.4e+06           270.0  270.0    0.0     0.0   1e+07  ...
```

## Running with MPI

FAST.Farm must be compiled with MPI support. Run your Python script with:

```bash
python your_script.py
```

The interface uses `MPI.COMM_SELF.Spawn()` to launch FAST.Farm, so you don't need `mpirun` for the Python side.

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| `MPI not initialized` | Ensure mpi4py is installed correctly |
| `Spawn failed` | Check FAST.Farm executable path and permissions |
| `Timeout waiting for measures` | FAST.Farm may have crashed; check its output |
| `Shape mismatch` | Verify `num_turbines` matches your .fstf file |

### Debugging Tips

1. Enable logging to see all communication:
   ```python
   interface = FastFarmInterface(..., log_file="debug.txt")
   ```

2. Check FAST.Farm console output for simulation errors

3. Verify your .fstf file runs correctly standalone:
   ```bash
   FAST.Farm your_simulation.fstf
   ```

## Class Diagram

```
BaseSimulatorInterface (ABC)
         │
         ▼
  FastFarmInterface
         │
         ├── uses ──► MPICommunicator (MPI send/receive)
         ├── uses ──► TurbineCommands (command buffers)
         ├── uses ──► CircularBuffer  (measurement history)
         └── uses ──► MeasureIndices  (data mapping)
```

## License

See the main project license.
