from utils import *


def Helbing_Model_2D(t, state, param, walls):
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


    ############################################################
    #Repulsive Force#
    ############################################################
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
    cos_theta = dot_product / (vel_magnitude+ 1e-9)

    # Apply 0.5 factor for agents behind (cos_theta < 0)
    angle_factor = np.where(cos_theta < 0, 1, 0.0)

    # Combine forces with angle factor
    force_magnitude = (repulsive_force_magnitude* angle_factor + body_force_magnitude) 

    # Calculate force components
    repulsive_force_x = np.sum(force_magnitude * eij_x + tangential_friction_magnitude * tij_x, axis=1)
    repulsive_force_y = np.sum(force_magnitude * eij_y + tangential_friction_magnitude * tij_y, axis=1)

    # Combine the x and y components into a single repulsive force vector
    repulsive_force = np.vstack((repulsive_force_x, repulsive_force_y))


    ############################################################
    #Wall Force#
    ############################################################
    wall_start_points = np.array([[wall[0], wall[1]] for wall in walls]).T  # Shape (2, num_walls)
    wall_end_points = np.array([[wall[2], wall[3]] for wall in walls]).T    # Shape (2, num_walls)
    wall_vectors = wall_end_points - wall_start_points  # Shape (2, num_walls)
    wall_lengths = np.linalg.norm(wall_vectors, axis=0)

    # Unit vectors along the walls
    wall_units = wall_vectors / wall_lengths

    # Initialize wall forces to zero
    wall_forces = np.zeros((2, N))

    # Calculate distances and forces from each wall to each agent
    for i in range(wall_units.shape[1]):
        # Vectors from wall start to agents
        wall_to_agent_vectors = positions - wall_start_points[:, i][:, np.newaxis]  # Shape (2, N)

        # Project wall-to-agent vectors onto the wall direction to find parallel components
        projection_lengths = np.dot(wall_units[:, i], wall_to_agent_vectors)  # Shape (N,)
        projection_lengths_clipped = np.clip(projection_lengths, 0, wall_lengths[i])

        # Closest points on the wall to each agent
        closest_points = wall_start_points[:, i][:, np.newaxis] + wall_units[:, i][:, np.newaxis] * projection_lengths_clipped

        # Perpendicular vectors from agents to closest points on the wall
        perp_vectors = positions - closest_points  # Shape (2, N)

        # Perpendicular distances from agents to the wall
        perp_distances = np.linalg.norm(perp_vectors, axis=0)  # Shape (N,)

        # Avoid division by zero by setting a minimum distance
        perp_distances = np.maximum(perp_distances, 1e-9)

        # Unit vectors in the direction of the perpendicular force (from wall to agent)
        e_w = perp_vectors / perp_distances

        # Tangential direction unit vector (perpendicular to e_w)
        t_w = np.array([-e_w[1, :], e_w[0, :]])  # Shape (2, N)

        # Wall repulsive force magnitude
        repulsive_force_magnitude_wall = A * np.exp((fixed_radius - perp_distances) / B)

        # Body compression force (only if agent is in contact with the wall)
        body_force_magnitude_wall = np.where(perp_distances < fixed_radius, k * (fixed_radius - perp_distances), 0)

        # Directional modulation based on velocity angle
        dot_product_wall = np.sum(velocities * e_w, axis=0)
        vel_magnitude_wall = np.linalg.norm(velocities, axis=0)
        cos_theta_wall = dot_product_wall / (vel_magnitude_wall + 1e-9)
        angle_factor_wall =  np.where(cos_theta_wall < 0, 1, 0.0)

        # Combine repulsive and body forces with angle factor
        force_magnitude_wall = (repulsive_force_magnitude_wall * angle_factor_wall + body_force_magnitude_wall)

        # Tangential friction force (only if agent is in contact with the wall)
        vel_diff_tangential_wall = np.sum(velocities * t_w, axis=0)
        tangential_friction_magnitude_wall = np.where(perp_distances < fixed_radius, -k * (fixed_radius - perp_distances) * vel_diff_tangential_wall, 0)

        # Calculate force components
        wall_forces += force_magnitude_wall * e_w + tangential_friction_magnitude_wall * t_w



    # Total acceleration (driving force + repulsive force)
    total_force = f_drv + repulsive_force + wall_forces
    accelerations = total_force / m

    # Update positions and velocities using Euler's method
    new_positions = velocities  # New positions: x + dx, y + dy
    new_velocities = accelerations  # New velocities: dx + ax, dy + ay

    # Stack updated state
    new_state = np.vstack([new_positions, new_velocities])

    return new_state, 1
