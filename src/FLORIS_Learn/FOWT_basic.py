'''
Basic turbine repositioning using FLORIS.
This is a simple example of how to use FLORIS to reposition a floating wind turbine based on the mooring system's response to wind loads.
The code calculates the mooring matrix for a given set of mooring parameters and then uses this matrix to determine the new position of the turbine under different wind conditions.
'''
import time
from typing import Tuple
import numpy as np
import matplotlib.pyplot as plt
from mooring_matrix import moor_matrix
from floris import FlorisModel
import floris.layout_visualization as layoutvisual
from floris.flow_visualization import visualize_cut_plane

# Turbine parameters for thrust calculation
ROTOR_DIAMETER = 125.88  # NREL 5MW [m]
ROTOR_AREA = np.pi * (ROTOR_DIAMETER / 2) ** 2
AIR_DENSITY = 1.225  # [kg/m^3]
N_TURBINES = 2
N_ITERATIONS = 10


def time_model_calculation(model_fmodel: FlorisModel, n_findex: int) -> Tuple[float, float]:
    """
    This function performs the wake calculation for a given
    FlorisModel object and computes the AEP while
    tracking the amount of wall-time required for both steps.

    Args:
        model_fmodel (FlorisModel): _description_
        float (_type_): _description_

    Returns:
        tuple(float, float):
            0: AEP
            1: Wall-time for the computation
    """
    start = time.perf_counter()
    model_fmodel.run()
    aep = model_fmodel.get_farm_power().sum() / n_findex  / 1E9 * 365 * 24
    end = time.perf_counter()
    return aep, end - start

# Create FLORIS instance
fmodel_floating = FlorisModel("Inputs/emgauss_floating_2T.yaml")

# Get mooring matrix instance
moor = moor_matrix()
moor.get_mooring_matrix(visualize=True)

# Set wind conditions
wind_direction = 270.0
wind_speed = 12.0
turbulence_intensity = 0.06

fmodel_floating.set(
    wind_directions=[wind_direction],
    wind_speeds=[wind_speed],
    turbulence_intensities=[turbulence_intensity],
)

# Yaw command for the turbine [deg]
yaw_commands = np.array([[20.0, -10.0]])

# Run and reposition the turbine based on the mooring matrix and yaw commands
x_orig = np.array([0.0, 630.0])  # Original positions of the turbines
y_orig = np.array([0.0, 0.0])
position = np.zeros((4, N_TURBINES)) # [surge, sway, pitch, yaw] for each turbine

# ===================== Iterative coupling loop =====================
# Each iteration: reposition turbines -> run FLORIS -> compute thrust -> query mooring table
# Repeats until platform positions converge

for i in range(N_ITERATIONS):
    # 1. Shift turbine positions by mooring-induced surge and sway
    x_repositioned = x_orig + position[0]
    y_repositioned = y_orig + position[1]
    fmodel_floating.set(layout_x=x_repositioned, layout_y=y_repositioned)

    # 2. Effective yaw = commanded yaw + platform yaw displacement (rad -> deg)
    effective_yaw = -yaw_commands + position[3] * 180 / np.pi
    fmodel_floating.set(yaw_angles=effective_yaw.reshape(1, N_TURBINES))

    # 3. Run FLORIS simulation
    fmodel_floating.run()

    # 4. Calculate thrust force on each turbine: T = 0.5 * rho * A * Ct * V^2
    wind_speeds_at_turbines = fmodel_floating.turbine_average_velocities  # (1, N)
    Ct = fmodel_floating.get_turbine_thrust_coefficients()                # (1, N)
    thrusts = 0.5 * AIR_DENSITY * ROTOR_AREA * Ct * wind_speeds_at_turbines ** 2

    # 5. Decompose thrust into X-Y force components based on wind direction and yaw
    theta = (-wind_direction + 270 - yaw_commands) * np.pi / 180
    Fx = thrusts * np.cos(theta)
    Fy = thrusts * np.sin(theta)

    # 6. Query mooring lookup table: forces -> platform displacements
    #    Returns [surge(m), sway(m), pitch(rad), yaw(rad)] per turbine
    position = moor.get_position((Fx.flatten(), Fy.flatten()))

    # Print iteration progress
    power_kW = fmodel_floating.get_turbine_powers().flatten() / 1e3
    print(f"Iteration {i+1}/{N_ITERATIONS}:")
    print(f"  Surge:  {position[0]} m")
    print(f"  Sway:   {position[1]} m")
    print(f"  Pitch:  {np.rad2deg(position[2])} deg")
    print(f"  Yaw:    {np.rad2deg(position[3])} deg")
    print(f"  Power:  {power_kW} kW\n")

# ===================== Final results =====================
power = fmodel_floating.get_turbine_powers().flatten()              # [W]
wind_speeds_final = fmodel_floating.turbine_average_velocities.flatten()
Ct_final = fmodel_floating.get_turbine_thrust_coefficients().flatten()

print("=" * 60)
print("CONVERGED RESULTS")
print("=" * 60)
for t in range(N_TURBINES):
    thrust_kN = 0.5 * AIR_DENSITY * ROTOR_AREA * Ct_final[t] * wind_speeds_final[t] ** 2 / 1e3
    print(f"\n  Turbine {t+1}:")
    print(f"    Yaw command:       {yaw_commands.flatten()[t]:.1f} deg")
    print(f"    Wind speed:        {wind_speeds_final[t]:.2f} m/s")
    print(f"    Thrust coeff (Ct): {Ct_final[t]:.4f}")
    print(f"    Thrust force:      {thrust_kN:.1f} kN")
    print(f"    Power output:      {power[t]/1e3:.1f} kW")
    print(f"    Platform surge:    {position[0][t]:.2f} m")
    print(f"    Platform sway:     {position[1][t]:.2f} m")
    print(f"    Platform pitch:    {np.rad2deg(position[2][t]):.3f} deg")
    print(f"    Platform yaw:      {np.rad2deg(position[3][t]):.3f} deg")

print(f"\n  Total farm power:    {np.sum(power)/1e3:.1f} kW")
print("=" * 60)

time_taken = time_model_calculation(fmodel_floating, n_findex=1)    # n_findex is number of wind conditions (1 in this case)
print(f"\n  Time taken for final FLORIS run: {time_taken[1]:.2f} seconds")

# ===================== Visualization =====================
# Compare original layout vs repositioned layout after mooring response
x_final = x_orig + position[0]
y_final = y_orig + position[1]

horizontal_plane = fmodel_floating.calculate_horizontal_plane(height=90.0)
visualize_cut_plane(horizontal_plane)
layoutvisual.plot_turbine_labels(fmodel_floating)
plt.show()