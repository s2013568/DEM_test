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
    tau, v0, Length, Width, periodic, A, B, delta_t = param

    # Extract positions and velocities
    positions = np.round(state[:2, :], decimals=9)  # positions = [x, y] for each agent
    velocities = np.round(state[2:, :], decimals = 9)  # velocities = [dx, dy] for each agent
    N = positions.shape[1]     # Number of agents

    # Desired velocity vector in the x direction (for simplicity)
    desired_velocity = np.array([[v0], [0]])

    # Driving force (vectorized for all agents)
    f_drv = (desired_velocity - velocities) / tau
    # Calculate pairwise position differences
    dx_matrix = positions[0, :, np.newaxis] - positions[0, np.newaxis, :]
    dy_matrix = positions[1, :, np.newaxis] - positions[1, np.newaxis, :]

 
    # Apply periodic boundary conditions if enabled
    if periodic:
        dx_matrix = (dx_matrix + 0.5 * Length) % Length - 0.5 * Length
        # dy_matrix = (dy_matrix + 0.5 * Width) % Width - 0.5 * Width
    # Compute pairwise distances and direction unit vectors
    dist_matrix = np.sqrt(dx_matrix**2 + dy_matrix**2)
    
    np.fill_diagonal(dist_matrix, np.inf)  # Avoid self-interaction by setting diagonal to infinity
    # Calculate velocity magnitudes and direction vectors
    v_b = np.sqrt(np.sum(velocities**2, axis=0))  # Velocity magnitude of each agent
    print(v_b)

    v_b_delta_t = v_b * delta_t                   # Precompute v_b * delta_t
    eij_x = dx_matrix / dist_matrix               # x-component of unit vector between agents
    eij_y = dy_matrix / dist_matrix            # y-component of unit vector between agents

    # Calculate directional term w based on angle
    # Dot product between velocity vector and separation unit vector
    dot_product = np.where(
        v_b[:, np.newaxis] == 0.0,
        1,  # If v_b is zero, set dot product to 1
        (velocities[0, :, np.newaxis] * eij_x + velocities[1, :, np.newaxis] * eij_y) / v_b[:, np.newaxis]
    )

    # Cosine threshold for 200 degrees (cosine of -0.94)
    w = np.where(dot_product > -0.94, 1, 0.5)

    # Compute projected distance
    term1 = dist_matrix
    term2 = np.sqrt((dx_matrix - v_b_delta_t * eij_x)**2 + (dy_matrix - v_b_delta_t * eij_y)**2)
    print((term1 + term2)**2 - (v_b_delta_t[:, np.newaxis])**2)
    projected_dist_matrix = np.sqrt((term1 + term2)**2 - (v_b_delta_t[:, np.newaxis])**2)


    # Compute repulsive force magnitudes for each pair with directional term w
    force_magnitude = A * np.exp(-projected_dist_matrix / B) * w

    # Vectorized summation of repulsive force components
    repulsive_force_x = np.sum(force_magnitude * eij_x, axis=1)
    repulsive_force_y = np.sum(force_magnitude * eij_y, axis=1)

    # Combine the x and y components into a single repulsive force vector
    repulsive_force = np.vstack((repulsive_force_x, repulsive_force_y))
    


    # Total acceleration (driving force + repulsive force)
    total_force = f_drv + repulsive_force
    accelerations = total_force

    # Update positions and velocities using Euler's method
    new_positions = velocities  # New positions: x + dx, y + dy
    new_velocities = accelerations  # New velocities: dx + ax, dy + ay



    # Apply periodic boundary conditions if needed
    # if periodic:
    #     new_positions[0, :] %= Length  # Wrap x positions
        # new_positions[1, :] %= Width   # Wrap y positions

    # Stack updated state
    new_state = np.vstack([new_positions, new_velocities])
    
    return new_state, 1