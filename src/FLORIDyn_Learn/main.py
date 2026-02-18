'''
Basic 2-turbine FLORIDyn simulation.
Two IEA 10MW turbines in a row at 5D spacing, constant 12 m/s wind from west.
This is the dynamic equivalent of the FOWT_basic.py FLORIS case.
'''
import os
import logging
import time
import numpy as np
import matplotlib.pyplot as plt

logging.basicConfig(level=logging.ERROR)

import off.off as off
import off.off_interface as offi

def my_control(turbines, t):
    """
    Define your yaw control logic here.

    Parameters
    ----------
    turbines : list
        List of Turbine objects. Useful state per turbine:
            turbine.ambient_states.get_wind_dir_ind(0)          -> wind direction [deg]
            turbine.ambient_states.get_turbine_wind_speed_abs() -> wind speed [m/s]
            turbine.get_yaw_orientation()                       -> current orientation [deg]
            turbine.calc_yaw(wind_dir)                          -> current yaw misalignment [deg]
    t : float
        Current simulation time [s]

    Returns
    -------
    list[float]
        Desired yaw misalignment [deg] for each turbine.
        Positive = nacelle rotates clockwise away from wind.
    """
    yaw_commands = []
    if t < 20.0:
        # First 30s: no yawing, let wakes develop
        yaw_commands = [0.0 for _ in turbines]
    else:
        for i, turbine in enumerate(turbines):
            if i == 0:
                yaw_commands.append(20.0)   # steer T0 wake by 20 deg
            else:
                yaw_commands.append(0.0)   # keep T1 aligned with wind
    return yaw_commands


def main():
    start_time = time.time()

    # --- Step 1: Create OFF interface and initialize simulation ---
    oi = offi.OFFInterface()

    # Use the local 2-turbine config
    # Options: 'run_2T.yaml' (IEA 10MW) or 'run_2T_NREL5MW.yaml' (NREL 5MW)
    config_path = os.path.join(os.path.dirname(__file__), 'Inputs', 'run_2T_NREL5MW.yaml')
    oi.init_simulation_by_path(config_path)

    # --- Step 1b: Attach your control function ---
    oi.off_sim.controller.set_control_fn(my_control)

    # --- Step 2: Run the dynamic simulation ---
    print("Running FLORIDyn simulation (2 turbines, 120s)...")
    oi.run_sim()

    elapsed = time.time() - start_time
    print(f"Simulation completed in {elapsed:.2f} seconds\n")

    # --- Step 3: Extract and display results ---
    measurements = oi.measurements

    # Get unique turbine indices and time steps
    turbine_ids = sorted(measurements['t_idx'].unique())
    time_steps = sorted(measurements['time'].unique())

    print("=" * 60)
    print("SIMULATION RESULTS")
    print("=" * 60)

    for t_idx in turbine_ids:
        t_data = measurements[measurements['t_idx'] == t_idx]
        avg_power = t_data['power_OFF'].mean() / 1e6  # MW
        final_power = t_data['power_OFF'].iloc[-1] / 1e6
        print(f"\n  Turbine {int(t_idx)+1}:")
        print(f"    Average power: {avg_power:.2f} MW")
        print(f"    Final power:   {final_power:.2f} MW")

    total_avg = measurements.groupby('time')['power_OFF'].sum().mean() / 1e6
    print(f"\n  Average total farm power: {total_avg:.2f} MW")
    print("=" * 60)

    # --- Step 4: Plot power time series ---
    fig, ax = plt.subplots(figsize=(10, 5))
    for t_idx in turbine_ids:
        t_data = measurements[measurements['t_idx'] == t_idx].sort_values('time')
        ax.plot(t_data['time'], t_data['power_OFF'] / 1e6,
                label=f'WT-{int(t_idx)+1}', linewidth=2)

    # Also plot total farm power
    farm_power = measurements.groupby('time')['power_OFF'].sum().reset_index()
    ax.plot(farm_power['time'], farm_power['power_OFF'] / 1e6,
            label='Total Farm', linewidth=2, linestyle='--', color='black')

    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Power (MW)', fontsize=12)
    ax.set_title('FLORIDyn: 2-Turbine Dynamic Simulation', fontsize=14)
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

    # --- Step 5: Store outputs ---
    oi.store_measurements()
    oi.store_applied_control()
    print(f"\nResults saved to: {oi.off_sim.sim_dir}")


if __name__ == "__main__":
    main()
