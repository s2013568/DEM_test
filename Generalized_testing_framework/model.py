from utils import *


def Helbing_Model_2D(t, state, param, walls, mode = 'Line_Method'):
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
    alt_k = 2 * k

    # Extract positions and velocities
    positions = np.round(state[:2, :], decimals=9)  # positions = [x, y] for each agent
    velocities = np.round(state[2:, :], decimals=9)  # velocities = [dx, dy] for each agent
    N = positions.shape[1]  # Number of agents

    # Desired velocity vector in the x direction (for simplicity)
    if mode == 'default':
        desired_velocity = np.array([[v0], [0]])
    elif mode == 'Rounding_Corner':
        guide_line_start = np.array([50, -2])
        guide_line_end = np.array([60, -2])

        desired_velocity = np.zeros_like(velocities)
        for i in range(N):
            agent_pos = positions[:, i]

            if agent_pos[0] > 51:
                # Agent has passed point (51, 0), set desired velocity to (0, -v0)
                desired_velocity[:, i] = np.array([0, -v0])
            else:
                # Find vector between agent and point (50, 0)
                vector_to_point = np.array([51, 0]) - agent_pos

                # Extrapolate the vector and check if it intersects the guide line
                extrapolated_end = agent_pos + 10 * vector_to_point  # Extend the vector
                if line_intersects(agent_pos, extrapolated_end, guide_line_start, guide_line_end):
                    # Set desired velocity direction to the guide line
                    desired_velocity[:, i] = vector_to_point / np.linalg.norm(vector_to_point) * v0
                else:
                    # Set desired velocity to (v0, 0)
                    desired_velocity[:, i] = np.array([v0, 0])
    elif mode == 'Entering':
        guide_line_start = np.array([49.5, 3.6])
        guide_line_end = np.array([49.5, 6.4])

        desired_velocity = np.zeros_like(velocities)
        for i in range(N):
            agent_pos = positions[:, i]

            if agent_pos[0] > 49.5:
                # Agent has passed x = 50, set desired velocity to (v0, 0)
                desired_velocity[:, i] = np.array([v0, 0])
            else:
                # Calculate the shortest distance from agent to the guide line
                line_vector = guide_line_end - guide_line_start
                line_length = np.linalg.norm(line_vector)
                line_unit_vector = line_vector / line_length

                agent_to_line_start = agent_pos - guide_line_start
                projection_length = np.dot(agent_to_line_start, line_unit_vector)
                projection_length_clipped = np.clip(projection_length, 0, line_length)

                closest_point = guide_line_start + projection_length_clipped * line_unit_vector
                vector_to_closest_point = closest_point - agent_pos

                # Set desired velocity towards the closest point on the guide line
                desired_velocity[:, i] = vector_to_closest_point / np.linalg.norm(vector_to_closest_point) * v0

    elif mode == "Bi-Directional Flow":
        if 'bi_directional_velocity' not in Helbing_Model_2D.__dict__:
            desired_velocity = np.zeros_like(velocities)
            # Randomly choose half of agents desired velocity to be np.array([[v0], [0]]) and other half to be np.array([[-v0], [0]])
            half_N = N // 2
            desired_velocity[:, :half_N] = np.array([[v0], [0]])
            desired_velocity[:, half_N:] = np.array([[-v0], [0]])
            # Shuffle the desired velocities to ensure randomness
            indices = np.arange(N)
            np.random.shuffle(indices)
            desired_velocity = desired_velocity[:, indices]
            Helbing_Model_2D.bi_directional_velocity = desired_velocity
        else:
            desired_velocity = Helbing_Model_2D.bi_directional_velocity

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
    tangential_friction_magnitude = np.where(dist_matrix < radii_sum, alt_k * (radii_sum - dist_matrix) * vel_diff_tangential, 0)

    # Calculate angle between agent's velocity direction and eij
    dot_product = velocities[0, :, np.newaxis] * eij_x + velocities[1, :, np.newaxis] * eij_y
    vel_magnitude = np.sqrt(velocities[0, :]**2 + velocities[1, :]**2).reshape(-1, 1)
    cos_theta = dot_product / (vel_magnitude + 1e-9)
    # Ensure cos_theta is clipped within [-1, 1] to prevent issues with floating-point precision
    cos_theta = np.clip(cos_theta, -1, 1)

    # Calculate angle factor based on cosine of angle
    angle_factor = 0.5 * (1 - cos_theta)

    # print(angle_factor)

    # angle_factor = np.where(cos_theta < 0, 1, 0.5)


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
    # print(repulsive_force)
    accelerations = total_force / m

    # Update positions and velocities using Euler's method
    new_positions = velocities  # New positions: x + dx, y + dy
    new_velocities = accelerations  # New velocities: dx + ax, dy + ay

    # Stack updated state
    new_state = np.vstack([new_positions, new_velocities])

    return new_state, 1
