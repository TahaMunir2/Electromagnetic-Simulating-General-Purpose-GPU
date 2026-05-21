"""
Python Reference Implementation for 1D FDTD Solver

This module provides a reference implementation of the 1D FDTD algorithm
using Q3.13 fixed-point precision on a 64-cell Yee grid.

The implementation demonstrates:
- 1D Ey and Bz field components
- Hard source excitation with sine wave
- Fixed-point arithmetic (Q3.13)
- Boundary conditions (zero at edges)
"""

import numpy as np
from typing import Tuple, List

# Q3.13 Fixed-Point Parameters
Q_INTEGER_BITS = 3
Q_FRACTION_BITS = 13
Q_SCALE = 2 ** Q_FRACTION_BITS


def float_to_q313(value: float) -> int:
    """Convert float to Q3.13 fixed-point integer."""
    return int(np.clip(value * Q_SCALE, -(2**15), 2**15 - 1))


def q313_to_float(value: int) -> float:
    """Convert Q3.13 fixed-point integer to float."""
    return value / Q_SCALE


def q313_saturate(value: int) -> int:
    """Clamp an integer to signed 16-bit Q3.13 range."""
    return int(np.clip(int(value), -(2**15), 2**15 - 1))


class FDTD1DSolver:
    """1D FDTD Solver with Q3.13 Fixed-Point Arithmetic"""
    
    def __init__(self, num_cells: int = 64, source_freq: float = 1e9, dt: float = 1e-12):
        """
        Initialize the FDTD solver.
        
        Args:
            num_cells: Number of cells in Yee grid
            source_freq: Source frequency in Hz
            dt: Time step in seconds
        """
        self.num_cells = num_cells
        self.dt = dt
        self.source_freq = source_freq
        self.omega = 2 * np.pi * source_freq
        
        # Field arrays (in Q3.13)
        self.ey = np.zeros(num_cells, dtype=np.int32)  # Electric field
        self.bz = np.zeros(num_cells, dtype=np.int32)  # Magnetic field
        
        # FDTD coefficients (pre-calculated in Q3.13)
        # These depend on material parameters and time step
        self.ce = float_to_q313(0.5)  # Coefficient for Ey update
        self.cm = float_to_q313(0.5)  # Coefficient for Bz update
        
        self.time_step = 0
        self.history_ey = []
        self.history_bz = []
    
    def q313_multiply(self, a: int, b: int) -> int:
        """Multiply two Q3.13 numbers."""
        product = (int(a) * int(b)) >> Q_FRACTION_BITS
        return q313_saturate(product)
    
    def source_signal(self, time: float) -> float:
        """
        Generate hard source signal (sine wave).
        
        Args:
            time: Current simulation time
            
        Returns:
            Source value in Q3.13
        """
        return float_to_q313(0.5 * np.sin(self.omega * time))
    
    def update_step(self, source_value: int, source_index: int = 32):
        """
        Execute one FDTD time step.
        
        Args:
            source_value: Hard source input (Q3.13)
            source_index: Location of hard source in grid
        """
        # Temporary arrays for updates
        ey_new = self.ey.copy()
        bz_new = self.bz.copy()
        
        # Update Ey field
        for k in range(1, self.num_cells - 1):
            dh = bz_new[k + 1] - bz_new[k]
            delta_h = self.q313_multiply(self.ce, dh)
            ey_new[k] = q313_saturate(int(self.ey[k]) + delta_h)
        
        # Apply hard source
        ey_new[source_index] = source_value
        
        # Update Bz field
        for k in range(self.num_cells - 1):
            de = ey_new[k] - ey_new[k + 1]
            delta_e = self.q313_multiply(self.cm, de)
            bz_new[k] = q313_saturate(int(self.bz[k]) + delta_e)
        
        # Boundary conditions (zero at edges)
        ey_new[0] = 0
        ey_new[-1] = 0
        bz_new[0] = 0
        bz_new[-1] = 0
        
        self.ey = ey_new
        self.bz = bz_new
        
        # Store history
        self.history_ey.append(self.ey.copy())
        self.history_bz.append(self.bz.copy())
        
        self.time_step += 1
    
    def simulate(self, num_steps: int, source_index: int = 32) -> Tuple[np.ndarray, np.ndarray]:
        """
        Run simulation for specified number of steps.
        
        Args:
            num_steps: Number of time steps
            source_index: Location of hard source
            
        Returns:
            Tuple of (Ey history, Bz history)
        """
        for step in range(num_steps):
            time = step * self.dt
            source_val = self.source_signal(time)
            self.update_step(source_val, source_index)
        
        return np.array(self.history_ey), np.array(self.history_bz)


if __name__ == "__main__":
    # Example simulation
    solver = FDTD1DSolver(num_cells=64, source_freq=1e9, dt=1e-12)
    
    # Run 500 time steps
    ey_history, bz_history = solver.simulate(num_steps=500, source_index=32)
    
    print(f"Simulation completed: {solver.time_step} steps")
    print(f"Final Ey range: [{ey_history[-1].min()}, {ey_history[-1].max()}]")
    print(f"Final Bz range: [{bz_history[-1].min()}, {bz_history[-1].max()}]")
    
    # Convert to float for visualization
    ey_float = np.array([[q313_to_float(v) for v in row] for row in ey_history])
    bz_float = np.array([[q313_to_float(v) for v in row] for row in bz_history])
    
    print(f"\nFinal Ey (float) range: [{ey_float[-1].min():.6f}, {ey_float[-1].max():.6f}]")
    print(f"Final Bz (float) range: [{bz_float[-1].min():.6f}, {bz_float[-1].max():.6f}]")
