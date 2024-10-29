from utils import *


def Helbing_Model_2D(t, state, param):
    """
    Fully vectorized Helbing model for 2D simulation with driving force in the x-direction.
    
    Parameters:
        t (float): Current time (not used in this basic model).
        state (np.ndarray): The state matrix where:
                            state[0, :] = x positions,
                            state[1, :] = y positions,
                            state[2, :] = x velocities,
                            state[3, :] = y velocities.
        param (tuple): Parameters for the model:
                       (tau, v0, Length, Width, periodic).
    
    Returns:
        new_state (np.ndarray): Updated state matrix with updated positions and velocities.
        int: Placeholder return value (1).
    """
    # Unpack parameters
    tau, v0, Length, Width, periodic = param

    # Extract positions and velocities as vectors
    positions = state[:2, :]  # positions = [x, y] for each agent
    velocities = state[2:, :]  # velocities = [dx, dy] for each agent

    # Desired velocity vector in x direction only (for simplicity)
    desired_velocity = v0 * np.array([[1], [0]])

    # Driving force (vectorized for all agents)
    f_drv = (desired_velocity - velocities) / tau

    # Update accelerations, which is just the driving force in this simple model
    accelerations = f_drv

    # Update positions and velocities using Euler's method
    new_positions = positions + velocities  # New positions: x + dx, y + dy
    new_velocities = velocities + accelerations  # New velocities: dx + ax, dy + ay

    # Apply periodic boundary conditions if needed
    if periodic:
        new_positions[0, :] %= Length  # Wrap x positions
        new_positions[1, :] %= Width   # Wrap y positions

    # Stack updated state
    new_state = np.vstack([new_positions, new_velocities])
    
    return new_state, 1