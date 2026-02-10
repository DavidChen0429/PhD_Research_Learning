"""
FFPythonConnect.py - FAST.Farm Python Interface via MPI

This module provides a clean interface for bidirectional communication between
Python and FAST.Farm simulator using MPI (Message Passing Interface).

Communication Flow:
    1. Python spawns FAST.Farm as a child process
    2. Initial handshake exchanges configuration (num_measures, max_iterations)
    3. Per-iteration loop:
       - Python sends control commands (yaw, pitch, torque)
       - FAST.Farm executes simulation step
       - FAST.Farm sends back measurements (power, wind, loads, etc.)

Author: Reorganized from wfcrl interface.py
"""

import platform
import warnings
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Dict, List, Optional, Union

import numpy as np
import pandas as pd
from mpi4py import MPI


# =============================================================================
# Constants and Configuration
# =============================================================================

class MPITags(IntEnum):
    """MPI message tags for identifying different message types."""
    COMMUNICATION = 0  # Initial handshake / setup
    YAW = 1            # Yaw angle commands
    PITCH = 2          # Pitch angle commands
    TORQUE = 3         # Torque commands
    MEASURES = 4       # Measurement data from simulator


@dataclass
class MeasureIndices:
    """
    Maps measurement names to their indices in the data array received from FAST.Farm.

    The simulator sends a flattened array of measurements for each turbine.
    This class defines which index corresponds to which measurement type.
    """
    wind_speed: int = 0
    power: int = 1
    wind_direction: int = 2
    yaw: int = 3
    pitch: int = 4
    torque: int = 5
    load: List[int] = field(default_factory=lambda: [6, 7, 8, 9, 10, 11])

    def to_dict(self) -> Dict[str, Union[int, List[int]]]:
        """Convert to dictionary format."""
        return {
            "wind_speed": self.wind_speed,
            "power": self.power,
            "wind_direction": self.wind_direction,
            "yaw": self.yaw,
            "pitch": self.pitch,
            "torque": self.torque,
            "load": self.load,
        }

    @property
    def total_count(self) -> int:
        """Total number of measurement values per turbine."""
        return 6 + len(self.load)  # 6 scalar values + load components


# =============================================================================
# Data Buffer for Time-Series Storage
# =============================================================================

class CircularBuffer:
    """
    Circular buffer for storing time-series measurements.

    Efficiently stores a rolling window of measurements, automatically
    overwriting oldest data when capacity is reached.

    Attributes:
        capacity: Maximum number of entries to store
        num_columns: Number of values per entry (e.g., num_turbines for power)
        position: Current write position in buffer
    """

    def __init__(
        self,
        num_columns: int,
        capacity: int = 50_000,
        aggregation_fn=np.mean
    ):
        """
        Initialize the circular buffer.

        Args:
            num_columns: Number of columns (e.g., number of turbines)
            capacity: Maximum number of rows to store
            aggregation_fn: Function to aggregate values (default: np.mean)
        """
        self._capacity = capacity
        self._num_columns = num_columns
        self._aggregation_fn = aggregation_fn
        self._buffer = np.zeros((capacity, num_columns), dtype=np.float32)
        self._position = -1  # No data yet

    def add(self, values: np.ndarray) -> None:
        """
        Add a new measurement row to the buffer.

        Args:
            values: Array of shape (num_columns,) to add
        """
        if self._position < self._capacity - 1:
            self._position += 1
        else:
            # Buffer full - shift data left (discard oldest)
            self._buffer = np.roll(self._buffer, -1, axis=0)

        self._buffer[self._position, :] = values

    def get_latest(self) -> np.ndarray:
        """Get the most recent measurement."""
        return self._buffer[self._position, :]

    def get_window(self, window_size: int = 1) -> np.ndarray:
        """
        Get the last N measurements.

        Args:
            window_size: Number of recent entries to return

        Returns:
            Array of shape (window_size, num_columns)
        """
        start = max(0, self._position - window_size + 1)
        return self._buffer[start:self._position + 1, :]

    def get_aggregated(self, window_size: int = 1) -> np.ndarray:
        """
        Get aggregated value over the last N measurements.

        Args:
            window_size: Number of entries to aggregate

        Returns:
            Aggregated values of shape (num_columns,)
        """
        data = self.get_window(window_size)
        return self._aggregation_fn(data, axis=0)

    def clear(self) -> None:
        """Reset the buffer to empty state."""
        self._buffer[:] = 0.0
        self._position = -1

    @property
    def is_empty(self) -> bool:
        """Check if buffer has no data."""
        return self._position < 0


# =============================================================================
# Command Container
# =============================================================================

@dataclass
class TurbineCommands:
    """
    Container for turbine control commands.

    Each command array has format: [active_flag, turbine1, turbine2, ...]
    - active_flag = 1.0 means the command should be applied
    - active_flag = 0.0 means no command (use default behavior)

    All angles are stored in RADIANS internally.
    """
    num_turbines: int

    def __post_init__(self):
        # +1 for the active flag at index 0
        self._yaw = np.zeros(self.num_turbines + 1, dtype=np.float64)
        self._pitch = np.zeros(self.num_turbines + 1, dtype=np.float64)
        self._torque = np.zeros(self.num_turbines + 1, dtype=np.float64)

    def set_yaw(self, angles_deg: np.ndarray) -> None:
        """Set yaw angles in degrees (converted to radians internally)."""
        self._yaw[0] = 1.0  # Mark as active
        self._yaw[1:] = np.radians(angles_deg.astype(np.float64))

    def set_pitch(self, angles_deg: np.ndarray) -> None:
        """Set pitch angles in degrees (converted to radians internally)."""
        self._pitch[0] = 1.0
        self._pitch[1:] = np.radians(angles_deg.astype(np.float64))

    def set_torque(self, torques: np.ndarray) -> None:
        """Set torque values."""
        self._torque[0] = 1.0
        self._torque[1:] = torques.astype(np.float64)

    def get_yaw_degrees(self) -> Optional[np.ndarray]:
        """Get yaw angles in degrees, or None if not set."""
        if self._yaw[0] == 0.0:
            return None
        return np.degrees(self._yaw[1:])

    def get_pitch_degrees(self) -> Optional[np.ndarray]:
        """Get pitch angles in degrees, or None if not set."""
        if self._pitch[0] == 0.0:
            return None
        return np.degrees(self._pitch[1:])

    def get_torque(self) -> Optional[np.ndarray]:
        """Get torque values, or None if not set."""
        if self._torque[0] == 0.0:
            return None
        return self._torque[1:].copy()

    def reset(self) -> None:
        """Clear all commands."""
        self._yaw[:] = 0.0
        self._pitch[:] = 0.0
        self._torque[:] = 0.0

    @property
    def yaw_buffer(self) -> np.ndarray:
        """Raw yaw buffer for MPI sending."""
        return self._yaw

    @property
    def pitch_buffer(self) -> np.ndarray:
        """Raw pitch buffer for MPI sending."""
        return self._pitch

    @property
    def torque_buffer(self) -> np.ndarray:
        """Raw torque buffer for MPI sending."""
        return self._torque


# =============================================================================
# Abstract Base Interface
# =============================================================================

class BaseSimulatorInterface(ABC):
    """
    Abstract base class for wind farm simulator interfaces.

    Defines the common interface that all simulator backends must implement.
    """

    def __init__(self, num_turbines: int):
        self.num_turbines = num_turbines

    @property
    @abstractmethod
    def wind_speed(self) -> float:
        """Current wind speed [m/s]."""
        pass

    @property
    @abstractmethod
    def wind_direction(self) -> float:
        """Current wind direction [degrees]."""
        pass

    @abstractmethod
    def initialize(self) -> None:
        """Initialize or reset the simulator connection."""
        pass

    @abstractmethod
    def send_commands(
        self,
        yaw: Optional[np.ndarray] = None,
        pitch: Optional[np.ndarray] = None,
        torque: Optional[np.ndarray] = None,
    ) -> bool:
        """
        Send control commands and advance simulation.

        Args:
            yaw: Yaw angles in degrees for each turbine
            pitch: Pitch angles in degrees for each turbine
            torque: Torque values for each turbine

        Returns:
            True if simulation has reached max iterations, False otherwise
        """
        pass

    @abstractmethod
    def get_powers(self) -> np.ndarray:
        """Get current power output for each turbine [W]."""
        pass


# =============================================================================
# MPI Communication Handler
# =============================================================================

class MPICommunicator:
    """
    Handles low-level MPI communication with FAST.Farm.

    This class encapsulates all MPI send/receive operations, providing
    a clean interface for the higher-level FastFarmInterface.
    """

    def __init__(
        self,
        comm: MPI.Comm,
        target_rank: int = 0,
    ):
        """
        Initialize MPI communicator.

        Args:
            comm: MPI communicator (usually an Intercomm from Spawn)
            target_rank: Rank of the target process (usually 0)
        """
        self._comm = comm
        self._target_rank = target_rank

    def send_command(self, data: np.ndarray, tag: MPITags) -> None:
        """
        Send a command array to FAST.Farm.

        Args:
            data: NumPy array to send
            tag: MPI tag identifying the message type
        """
        self._comm.Send(buf=data, dest=self._target_rank, tag=tag)

    def receive_array(self, buffer: np.ndarray, tag: MPITags) -> None:
        """
        Receive data from FAST.Farm into a pre-allocated buffer.

        Args:
            buffer: Pre-allocated NumPy array to receive into
            tag: MPI tag to listen for
        """
        self._comm.Recv(buffer, source=self._target_rank, tag=tag)

    def synchronize(self) -> None:
        """Block until all processes reach this point."""
        self._comm.Barrier()

    def disconnect(self) -> None:
        """Disconnect from the intercommunicator."""
        if isinstance(self._comm, MPI.Intercomm):
            self._comm.Disconnect()

    @property
    def communicator(self) -> MPI.Comm:
        """Access the underlying MPI communicator."""
        return self._comm


# =============================================================================
# FAST.Farm Interface
# =============================================================================

class FastFarmInterface(BaseSimulatorInterface):
    """
    Interface for communicating with FAST.Farm wind farm simulator via MPI.

    This class manages the complete lifecycle of FAST.Farm communication:
    1. Spawning the FAST.Farm process
    2. Initial handshake (exchanging configuration)
    3. Sending control commands each iteration
    4. Receiving measurement data
    5. Clean shutdown

    Example Usage:
        >>> interface = FastFarmInterface(
        ...     num_turbines=3,
        ...     fstf_file="path/to/farm.fstf",
        ...     max_iterations=1000
        ... )
        >>> interface.initialize()
        >>>
        >>> for _ in range(1000):
        ...     # Send yaw commands
        ...     done = interface.send_commands(yaw=np.array([270, 275, 270]))
        ...
        ...     # Get results
        ...     powers = interface.get_powers()
        ...     wind = interface.get_wind_measurements()
        ...
        ...     if done:
        ...         break

    Attributes:
        num_turbines: Number of turbines in the wind farm
        measure_indices: Mapping of measurement names to array indices
    """

    # Detect platform for default executable path
    _SYSTEM = platform.system().lower()
    _DEFAULT_EXE = (
        "simulators/fastfarm/bin/FAST.Farm_x64_OMP_2023.exe"
        if _SYSTEM == "windows"
        else "FAST.Farm"
    )

    def __init__(
        self,
        num_turbines: int,
        fstf_file: str,
        max_iterations: int = 10_000,
        executable_path: str = None,
        buffer_capacity: int = 50_000,
        averaging_window: int = 1,
        log_file: Optional[str] = None,
    ):
        """
        Initialize FAST.Farm interface.

        Args:
            num_turbines: Number of turbines in the simulation
            fstf_file: Path to the FAST.Farm input file (.fstf)
            max_iterations: Maximum simulation iterations
            executable_path: Path to FAST.Farm executable (auto-detected if None)
            buffer_capacity: Size of measurement history buffer
            averaging_window: Default window size for averaging measurements
            log_file: Optional path for logging communication (None = no logging)
        """
        super().__init__(num_turbines)

        # Configuration
        self._fstf_file = Path(fstf_file)
        self._executable = executable_path or self._DEFAULT_EXE
        self._max_iterations = max_iterations
        self._averaging_window = averaging_window

        # Measurement configuration
        self.measure_indices = MeasureIndices()
        self._num_measures = self.measure_indices.total_count

        # State
        self._current_iteration = 0
        self._num_resets = 0
        self._communicator: Optional[MPICommunicator] = None

        # Data storage
        self._commands = TurbineCommands(num_turbines)
        self._power_buffer = CircularBuffer(num_turbines, buffer_capacity)
        self._wind_buffer = CircularBuffer(2, buffer_capacity)  # [speed, direction]
        self._current_measures = np.zeros((num_turbines, self._num_measures))

        # Logging
        self._log_file = log_file
        self._logging_enabled = log_file is not None

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def wind_speed(self) -> float:
        """Current averaged wind speed [m/s]."""
        return self._get_averaged_wind()[0]

    @property
    def wind_direction(self) -> float:
        """Current averaged wind direction [degrees]."""
        return self._get_averaged_wind()[1]

    @property
    def current_iteration(self) -> int:
        """Current simulation iteration number."""
        return self._current_iteration

    @property
    def is_finished(self) -> bool:
        """Check if simulation has reached max iterations."""
        return self._current_iteration >= self._max_iterations

    # -------------------------------------------------------------------------
    # Public Methods
    # -------------------------------------------------------------------------

    def initialize(
        self,
        wind_speed: Optional[float] = None,
        wind_direction: Optional[float] = None,
    ) -> None:
        """
        Initialize connection to FAST.Farm.

        Spawns the FAST.Farm process and performs initial handshake.

        Args:
            wind_speed: Optional wind speed override (if supported by inflow file)
            wind_direction: Optional wind direction override (currently not supported)
        """
        # Handle wind direction warning
        if wind_direction is not None:
            warnings.warn(
                f"Wind direction = {wind_direction} requested, but FastFarmInterface "
                "cannot set wind direction in the simulator. Request will be ignored."
            )

        # Update inflow file if wind speed specified
        if wind_speed is not None:
            self._update_inflow_wind_speed(wind_speed)

        # Reset state
        self._current_iteration = 0
        self._commands.reset()
        self._power_buffer.clear()
        self._wind_buffer.clear()

        # Spawn FAST.Farm process
        simul_file = self._prepare_simulation_file()
        spawn_comm = self._spawn_fastfarm_process(simul_file)
        self._communicator = MPICommunicator(spawn_comm, target_rank=0)
        self._num_resets += 1

        # Perform handshake
        self._perform_handshake()

    def send_commands(
        self,
        yaw: Optional[np.ndarray] = None,
        pitch: Optional[np.ndarray] = None,
        torque: Optional[np.ndarray] = None,
    ) -> bool:
        """
        Send control commands to FAST.Farm and receive measurements.

        This is the main interaction method. It:
        1. Prepares command buffers
        2. Sends commands via MPI
        3. Waits for and processes measurements
        4. Updates internal buffers

        Args:
            yaw: Yaw angles in degrees for each turbine
            pitch: Pitch angles in degrees for each turbine
            torque: Generator torque for each turbine

        Returns:
            True if simulation has reached max iterations, False otherwise

        Raises:
            RuntimeError: If called before initialize()
        """
        if self._communicator is None:
            raise RuntimeError("Must call initialize() before send_commands()")

        # Update command buffers
        if yaw is not None:
            self._commands.set_yaw(yaw)
        if pitch is not None:
            self._commands.set_pitch(pitch)
        if torque is not None:
            self._commands.set_torque(torque)

        # Send all commands to FAST.Farm
        self._send_all_commands()

        # Wait for and process measurements
        powers, wind = self._receive_measurements()
        self._power_buffer.add(powers)
        self._wind_buffer.add(wind)

        # Update iteration counter
        self._current_iteration += 1

        # Check if finished
        if self.is_finished:
            self._finalize()

        # Log if enabled
        self._log_iteration(powers, wind)

        return self.is_finished

    def get_powers(self, window: Optional[int] = None) -> np.ndarray:
        """
        Get averaged power output for each turbine.

        Args:
            window: Number of iterations to average over (default: averaging_window)

        Returns:
            Array of power values [W] for each turbine
        """
        window = window or self._averaging_window
        return np.atleast_1d(self._power_buffer.get_aggregated(window).squeeze())

    def get_total_farm_power(self, window: Optional[int] = None) -> float:
        """
        Get total farm power output.

        Args:
            window: Number of iterations to average over

        Returns:
            Sum of all turbine powers [W]
        """
        return float(self.get_powers(window).sum())

    def get_wind_measurements(self, window: Optional[int] = None) -> np.ndarray:
        """
        Get averaged wind measurements.

        Args:
            window: Number of iterations to average over

        Returns:
            Array of [wind_speed, wind_direction]
        """
        window = window or self._averaging_window
        return self._wind_buffer.get_aggregated(window).squeeze()

    def get_measurement(self, name: str) -> np.ndarray:
        """
        Get a specific measurement by name.

        Args:
            name: Measurement name (wind_speed, power, yaw, pitch, torque, load, etc.)

        Returns:
            Array of measurement values for each turbine
        """
        if name == "freewind_measurements":
            return np.atleast_1d(self._wind_buffer.get_latest().squeeze())

        indices = self.measure_indices.to_dict().get(name)
        if indices is None:
            raise ValueError(f"Unknown measurement: {name}")

        return np.atleast_1d(self._current_measures[:, indices].squeeze())

    def get_all_measurements(self) -> pd.DataFrame:
        """
        Get all current measurements as a DataFrame.

        Returns:
            DataFrame with measurements for each turbine (rows) and
            measurement types (columns)
        """
        # Build column names
        columns = ["wind_speed", "power", "wind_direction", "yaw", "pitch", "torque"]
        columns += [f"load_{i}" for i in range(len(self.measure_indices.load))]

        df = pd.DataFrame(self._current_measures, columns=columns)

        # Convert angles to degrees for readability
        df[["yaw", "pitch"]] = np.degrees(df[["yaw", "pitch"]])

        return df

    def get_yaw_command(self) -> Optional[np.ndarray]:
        """Get current yaw command in degrees, or None if not set."""
        return self._commands.get_yaw_degrees()

    def get_pitch_command(self) -> Optional[np.ndarray]:
        """Get current pitch command in degrees, or None if not set."""
        return self._commands.get_pitch_degrees()

    def get_torque_command(self) -> Optional[np.ndarray]:
        """Get current torque command, or None if not set."""
        return self._commands.get_torque()

    # -------------------------------------------------------------------------
    # Private Methods - MPI Communication
    # -------------------------------------------------------------------------

    def _spawn_fastfarm_process(self, simul_file: str) -> MPI.Intercomm:
        """
        Spawn FAST.Farm as a child MPI process.

        Args:
            simul_file: Path to the simulation file

        Returns:
            MPI Intercommunicator for bidirectional communication
        """
        print(f"Spawning FAST.Farm process: {self._executable} {simul_file}")

        return MPI.COMM_SELF.Spawn(
            self._executable,
            args=[simul_file],
            maxprocs=1
        )

    def _perform_handshake(self) -> None:
        """
        Perform initial handshake with FAST.Farm.

        Exchange configuration:
        - Receive: number of measurements per turbine
        - Send: maximum number of iterations
        """
        # Receive number of measurements from FAST.Farm
        num_measures_buffer = np.array([0], dtype=np.int32)
        self._communicator.receive_array(num_measures_buffer, MPITags.COMMUNICATION)

        received_num_measures = num_measures_buffer[0]
        print(f"Interface: will receive {received_num_measures} measures per iteration")

        # Send max iterations to FAST.Farm
        max_iter_buffer = np.array([self._max_iterations], dtype=np.float64)
        self._communicator.send_command(max_iter_buffer, MPITags.COMMUNICATION)

        # Initialize measurement storage
        self._current_measures = np.zeros(
            (self.num_turbines, received_num_measures),
            dtype=np.float64
        ) * np.nan

    def _send_all_commands(self) -> None:
        """Send yaw, pitch, and torque commands to FAST.Farm."""
        self._communicator.send_command(self._commands.yaw_buffer, MPITags.YAW)
        self._communicator.send_command(self._commands.pitch_buffer, MPITags.PITCH)
        self._communicator.send_command(self._commands.torque_buffer, MPITags.TORQUE)

    def _receive_measurements(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Receive measurement data from FAST.Farm.

        Returns:
            Tuple of (powers, wind) where:
            - powers: Array of power values for each turbine
            - wind: Array of [wind_speed, wind_direction]
        """
        # Receive raw data
        buffer_size = self.num_turbines * self._num_measures
        raw_measures = np.zeros(buffer_size, dtype=np.float64)

        self._communicator.receive_array(raw_measures, MPITags.MEASURES)
        self._communicator.synchronize()

        # Reshape to (num_turbines, num_measures)
        measures = raw_measures.reshape((self.num_turbines, self._num_measures))

        # Process wind directions (convert to standard format)
        dir_idx = self.measure_indices.wind_direction
        directions = np.degrees(measures[:, dir_idx].flatten()) - 90
        directions[directions < 0] += 360  # Keep positive
        measures[:, dir_idx] = directions

        # Extract values
        speed_idx = self.measure_indices.wind_speed
        power_idx = self.measure_indices.power

        speeds = measures[:, speed_idx].flatten()
        powers = measures[:, power_idx].flatten()

        # Determine upstream turbine (highest wind speed = least wake effect)
        upstream_idx = np.argmax(speeds)
        upstream_speed = speeds[upstream_idx]
        upstream_direction = directions[upstream_idx]

        # Store current measurements
        self._current_measures = measures

        return (
            powers.astype(np.float32),
            np.array([upstream_speed, upstream_direction], dtype=np.float32)
        )

    def _finalize(self) -> None:
        """Clean up MPI connection."""
        if self._communicator is not None:
            self._communicator.disconnect()

    # -------------------------------------------------------------------------
    # Private Methods - Helpers
    # -------------------------------------------------------------------------

    def _get_averaged_wind(self) -> np.ndarray:
        """Get averaged wind [speed, direction]."""
        return self._wind_buffer.get_aggregated(self._averaging_window).squeeze()

    def _prepare_simulation_file(self) -> str:
        """
        Prepare simulation file for this run.

        For multiple resets, may create modified copies of the input file.

        Returns:
            Path to the simulation file to use
        """
        # This is a placeholder - in the original code, reset_simul_file
        # handles creating modified simulation files for reruns
        # You may need to import and use that function here
        return str(self._fstf_file)

    def _update_inflow_wind_speed(self, wind_speed: float) -> None:
        """
        Update inflow file with new wind speed.

        Args:
            wind_speed: New wind speed value
        """
        # This is a placeholder - in the original code, this reads/writes
        # the inflow file. You may need to import the relevant functions
        # from simul_utils
        pass

    def _log_iteration(self, powers: np.ndarray, wind: np.ndarray) -> None:
        """Log iteration data if logging is enabled."""
        if not self._logging_enabled:
            return

        with open(self._log_file, "a") as fp:
            fp.write(
                f"Iteration {self._current_iteration}:\n"
                f"  Commands - YAW: {self.get_yaw_command()} | "
                f"PITCH: {self.get_pitch_command()} | "
                f"TORQUE: {self.get_torque_command()}\n"
                f"  Raw Power: {powers}\n"
                f"  Avg Power (window={self._averaging_window}): {self.get_powers()}\n"
                f"  Wind: speed={wind[0]:.2f} m/s, dir={wind[1]:.1f} deg\n"
                f"---\n"
            )


# =============================================================================
# Convenience Factory Function
# =============================================================================

def create_fastfarm_interface(
    fstf_file: str,
    executable: Optional[str] = None,
    **kwargs
) -> FastFarmInterface:
    """
    Create a FastFarmInterface from a .fstf file.

    This is a convenience function that automatically reads the number
    of turbines and max iterations from the simulation file.

    Args:
        fstf_file: Path to the FAST.Farm input file
        executable: Path to FAST.Farm executable (auto-detected if None)
        **kwargs: Additional arguments passed to FastFarmInterface

    Returns:
        Configured FastFarmInterface instance

    Example:
        >>> interface = create_fastfarm_interface("my_farm.fstf")
        >>> interface.initialize()
    """
    # Import the utility function to read simulation info
    # You'll need to ensure this import works in your environment
    try:
        from wfcrl.simul_utils import read_simul_info, create_dll

        num_turbines, max_iter = read_simul_info(fstf_file)
        print(f"Creating DLLs for simulation: {fstf_file}")
        create_dll(fstf_file)

    except ImportError:
        raise ImportError(
            "Could not import wfcrl.simul_utils. "
            "Please specify num_turbines and max_iterations manually."
        )

    return FastFarmInterface(
        num_turbines=num_turbines,
        fstf_file=fstf_file,
        max_iterations=max_iter,
        executable_path=executable,
        **kwargs
    )


# =============================================================================
# Example Usage
# =============================================================================

if __name__ == "__main__":
    """
    Example demonstrating basic usage of FastFarmInterface.

    Note: This requires a valid FAST.Farm installation and input files.
    """

    print("FastFarm Python Interface - Example")
    print("=" * 50)

    # Example configuration (update paths for your system)
    FSTF_FILE = "path/to/your/simulation.fstf"
    NUM_TURBINES = 3
    MAX_ITERATIONS = 100

    # Create interface
    interface = FastFarmInterface(
        num_turbines=NUM_TURBINES,
        fstf_file=FSTF_FILE,
        max_iterations=MAX_ITERATIONS,
        averaging_window=5,
        log_file="simulation_log.txt"
    )

    # Initialize (spawns FAST.Farm)
    print("\nInitializing connection to FAST.Farm...")
    # interface.initialize()  # Uncomment when you have valid input files

    # Example control loop
    print("\nRunning simulation loop...")
    """
    for i in range(MAX_ITERATIONS):
        # Calculate yaw commands (example: simple offset)
        yaw_commands = np.array([270.0, 275.0, 270.0])

        # Send commands and check if done
        finished = interface.send_commands(yaw=yaw_commands)

        # Get measurements
        powers = interface.get_powers()
        total_power = interface.get_total_farm_power()
        wind = interface.get_wind_measurements()

        # Print status every 10 iterations
        if i % 10 == 0:
            print(f"  Iter {i}: Total Power = {total_power/1e6:.2f} MW, "
                  f"Wind = {wind[0]:.1f} m/s @ {wind[1]:.0f} deg")

        if finished:
            print(f"\nSimulation completed after {i+1} iterations")
            break
    """

    print("\nExample complete. Modify paths and uncomment to run actual simulation.")
