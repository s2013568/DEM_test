import os
# from numpy import *
import numpy as np
import logging

def sh(script):
    # run bash command
    os.system("bash -c '%s'" % script)

#======================================================
def init(N, Length, Width):
    error = 0.0  # small positional error
    eps_x = 0  # small perturbation in x-direction
    eps_y = 0  # small perturbation in y-direction
    eps_vx = 0  # small perturbation in velocity x-direction
    eps_vy = 0  # small perturbation in y-direction

    if Width == 0:
        # Single line initialization
        shift_x = float(Length) / N
        x_positions = 0.5 * shift_x + shift_x * np.arange(N) + np.random.uniform(0, eps_x, N)
        y_positions = np.ones(N)
        positions = np.vstack([x_positions, y_positions])
    else:
        # 2D grid initialization
        grid_size = int(np.ceil(np.sqrt(N)))  # Determine the size of the grid
        shift_x = float(Length) / grid_size
        shift_y = float(Width) / grid_size

        x_n = 0.5 * shift_x + shift_x * np.arange(grid_size)
        y_n = 0.5 * shift_y + shift_y * np.arange(grid_size)

        xx, yy = np.meshgrid(x_n, y_n)
        positions = np.vstack([xx.ravel()[:N], yy.ravel()[:N]])

        # Add a small error to the first agent's position
        positions[0, 0] += error

    # Initialize velocities with small perturbations
    velocities = np.vstack([
        np.full(N, 0.00),
        np.full(N, 0.00)
    ])

    return np.vstack([positions, velocities])


#======================================================
def euler(x, h, y, f, param, walls):
    y_new, flag = f(x, y, param, walls)
    return x + h, y + h * y_new, flag

#======================================================
def get_state_vars(state, Length, Width, periodic=True):
    """
    Efficiently extracts state variables and calculates pairwise distances between agents in 2D.
    Each agent interacts with every other agent in the space.
    
    Parameters:
        state (np.ndarray): The state matrix where
                            state[0, :] = x positions,
                            state[1, :] = y positions,
                            state[2, :] = x velocities,
                            state[3, :] = y velocities.
        Length (float): The length of the simulation area.
        Width (float): The width of the simulation area.
        periodic (bool): Whether to apply periodic boundary conditions.
    
    Returns:
        x_n, y_n: Original positions of each agent.
        dx_n, dy_n: Velocities of each agent.
        dist_matrix: Matrix of distances between every pair of agents.
    """
    N = state.shape[1]  # Number of agents

    # Extract x and y positions and velocities
    x_n = state[0, :]
    y_n = state[1, :]
    dx_n = state[2, :]
    dy_n = state[3, :]

    # Create matrices for pairwise position differences
    dx_matrix = x_n[:, np.newaxis] - x_n[np.newaxis, :]
    dy_matrix = y_n[:, np.newaxis] - y_n[np.newaxis, :]

    if periodic:
        # Apply periodic boundary conditions by adjusting for nearest image convention
        dx_matrix = (dx_matrix + 0.5 * Length) % Length - 0.5 * Length
        dy_matrix = (dy_matrix + 0.5 * Width) % Width - 0.5 * Width

    # Calculate pairwise Euclidean distances
    dist_matrix = np.sqrt(dx_matrix**2 + dy_matrix**2)

    # Avoid self-interaction by setting diagonal to infinity
    np.fill_diagonal(dist_matrix, np.inf)

    # Ensure positions stay within bounds using modulo
    x_n %= Length
    y_n %= Width

    return x_n, y_n, dx_n, dy_n, dist_matrix