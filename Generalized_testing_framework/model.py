from utils import *


def Helbing_Model_2D(t, state, param):
    """
    Fully vectorized Helbing model with projected distance calculation for repulsive forces.
    Includes directional term w to modulate the repulsive force based on angle.
    
    Parameters:
        t (float): Current time (not used in this basic model).
        state (np.ndarray): The state matrix where:
                            state[0, :] = x positions,
                            state[1, :] = y positions,
                            state[2, :] = x velocities,
                            state[3, :] = y velocities.
        param (tuple): Parameters for the model:
                       (tau, v0, Length, Width, periodic, A, B, delta_t).
    
    Returns:
        new_state (np.ndarray): Updated state matrix.
        int: Placeholder return value (1).
        
    """
  
    # Unpack parameters
    m, tau, v0, Length, Width, periodic, A, B, delta_t, fixed_radius, k = param

    # Extract positions and velocities
    positions = np.round(state[:2, :], decimals=9)  # positions = [x, y] for each agent
    velocities = np.round(state[2:, :], decimals=9)  # velocities = [dx, dy] for each agent
    N = positions.shape[1]  # Number of agents

    # Desired velocity vector in the x direction (for simplicity)
    desired_velocity = np.array([[v0], [0]])

    # Driving force (vectorized for all agents)
    f_drv = m * (desired_velocity - velocities) / tau

    # Calculate pairwise position differences
    dx_matrix = positions[0, :, np.newaxis] - positions[0, np.newaxis, :]
    dy_matrix = positions[1, :, np.newaxis] - positions[1, np.newaxis, :]

    # Apply periodic boundary conditions if enabled
    if periodic:
        dx_matrix = (dx_matrix + 0.5 * Length) % Length - 0.5 * Length

    # Compute pairwise distances and direction unit vectors
    dist_matrix = np.sqrt(dx_matrix**2 + dy_matrix**2)
    np.fill_diagonal(dist_matrix, 1e30)  # Avoid self-interaction by setting diagonal to infinity

    # Unit vectors in the normal direction (from j to i)
    eij_x = dx_matrix / dist_matrix
    eij_y = dy_matrix / dist_matrix

    # Fixed radii sum
    radii_sum = 2 * fixed_radius  # Sum of fixed radii for two agents

    # Exponential repulsive force
    repulsive_force_magnitude = A * np.exp((radii_sum - dist_matrix) / B)

    # Body compression force (only if agents are in contact)
    body_force_magnitude = np.where(dist_matrix < radii_sum, k * (radii_sum - dist_matrix), 0)

    # Tangential velocity difference (for friction term)
    vel_diff_x = velocities[0, :, np.newaxis] - velocities[0, np.newaxis, :]
    vel_diff_y = velocities[1, :, np.newaxis] - velocities[1, np.newaxis, :]

    # Compute tangential direction unit vector (perpendicular to eij)
    tij_x = -eij_y
    tij_y = eij_x

    # Tangential velocity difference
    vel_diff_tangential = vel_diff_x * tij_x + vel_diff_y * tij_y


    # Tangential friction force (only if agents are in contact)
    tangential_friction_magnitude = np.where(dist_matrix < radii_sum, k * (radii_sum - dist_matrix) * vel_diff_tangential, 0)

    # Calculate angle between agent's velocity direction and eij
    dot_product = velocities[0, :, np.newaxis] * eij_x + velocities[1, :, np.newaxis] * eij_y
    vel_magnitude = np.sqrt(velocities[0, :]**2 + velocities[1, :]**2).reshape(-1, 1)
    cos_theta = dot_product / vel_magnitude

    # Apply 0.5 factor for agents behind (cos_theta < 0)
    angle_factor = np.where(cos_theta < 0, 0, 1)

    # Combine forces with angle factor
    force_magnitude = - (repulsive_force_magnitude* angle_factor + body_force_magnitude) 

    # Calculate force components
    repulsive_force_x = np.sum(force_magnitude * eij_x + tangential_friction_magnitude * tij_x, axis=1)
    repulsive_force_y = np.sum(force_magnitude * eij_y + tangential_friction_magnitude * tij_y, axis=1)

    # Combine the x and y components into a single repulsive force vector
    repulsive_force = np.vstack((repulsive_force_x, repulsive_force_y))

    # Total acceleration (driving force + repulsive force)
    total_force = f_drv + repulsive_force
    accelerations = total_force / m

    # Update positions and velocities using Euler's method
    new_positions = velocities  # New positions: x + dx, y + dy
    new_velocities = accelerations  # New velocities: dx + ax, dy + ay

    # Stack updated state
    new_state = np.vstack([new_positions, new_velocities])

    return new_state, 1
