import numpy as np
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

def apply_periodic_distance(agent_i_pos, agent_j_pos, env):
    """Calculate minimum separation between two agents with periodic boundaries."""
    dx = agent_j_pos[0] - agent_i_pos[0]
    dy = agent_j_pos[1] - agent_i_pos[1]

    # Apply periodic boundaries along the x-axis
    if abs(dx) > env.width / 2:
        dx = dx - np.sign(dx) * env.width

    # Apply periodic boundaries along the y-axis
    if abs(dy) > env.height / 2:
        dy = dy - np.sign(dy) * env.height

    return np.array([dx, dy])

def closest_distance_between_ellipses(agent1, agent2, env):
    # separation_unit_vector = normalize(apply_periodic_distance(agent1.position, agent2.position, env))
    separation_unit_vector = normalize(agent2.position - agent1.position)

    velocity_unit_vector1 = normalize(agent1.velocity)
    velocity_unit_vector2 = normalize(agent2.velocity)
    angle1 = np.arccos(np.dot(velocity_unit_vector1, separation_unit_vector))
    angle2 = np.arccos(np.dot(velocity_unit_vector2, separation_unit_vector))
    q1 = ((np.cos(angle1) ** 2) / (agent1.a ** 2)) + ((np.sin(angle1) ** 2) / (agent1.b ** 2))
    q2 = ((np.cos(angle2) ** 2) / (agent2.a ** 2)) + ((np.sin(angle2) ** 2) / (agent2.b ** 2))
   #  print(q1)
   #  print(q2)

    r1 = np.sqrt(1 / q1)
    r2 = np.sqrt(1 / q2)
    
    h = np.linalg.norm(apply_periodic_distance(agent1.position, agent2.position, env)) - r1 - r2
    
    return h

def calculate_angle(velocity):
    # Compute the angle in degrees from the velocity vector
    return np.degrees(np.arctan2(velocity[1], velocity[0]))


def calculate_shortest_distance_between_point_and_line(agent, line_points):
    point1 = np.array(line_points[0])
    point2 = np.array(line_points[1])
    agent_pos = agent.position
    line_vector = point2 - point1
    line_length_squared = np.dot(line_vector, line_vector)
    point_to_agent_vector = agent_pos - point1
    t = np.dot(point_to_agent_vector, line_vector) / line_length_squared
    t = max(0, min(1, t))
    nearest_point_on_line = point1 + t * line_vector
    d = nearest_point_on_line - agent_pos
    d_mag = np.linalg.norm(d)
    d_unit = d / d_mag
    return d_mag, normalize(line_vector), nearest_point_on_line, point1, point2
    
    
def det_numpy(A, B):
    # Stack vectors A and B as columns of a 2x2 matrix
    matrix = np.column_stack((A, B))
    # Compute the determinant of the 2x2 matrix
    return np.linalg.det(matrix)



def contact_distance_between_two_ellipse(agent1, agent2):
    k1 = normalize(agent1.velocity)
    b1 = agent1.b
    a1 = agent1.a
    eta = (a1 / b1) - 1
    T = (1 /b1) * (np.identity(2) + ((b1 / a1) - 1) * np.outer(k1, k1))
    Tinv = b1 * (np.identity(2) + eta * np.outer(k1, k1))
    
    
def calculate_closest_distance(agent1, agent2):
    """
    Calculate the closest distance between two ellipses in 2D space.

    Parameters:
        agent1, agent2: Two objects representing the ellipses, each having the following attributes:
            - a: Semi-major axis of the ellipse.
            - b: Semi-minor axis of the ellipse.
            - velocity: A vector representing the orientation of the ellipse (this gives the direction of the semi-major axis).
            - position: A vector representing the center of the ellipse.
    
    Returns:
        The closest distance between the centers of the two ellipses.
    """

    # Step 1: Extract ellipse parameters
    a1, b1 = agent1.a, agent1.b
    a2, b2 = agent2.a, agent2.b
    k1 = normalize(agent1.velocity)  # Orientation of ellipse 1
    k2 = normalize(agent2.velocity)  # Orientation of ellipse 2
    position_diff = agent2.position - agent1.position  # Vector joining the centers of the ellipses

    # Step 2: Define the transformation matrix to transform ellipse 1 into a unit circle
    def transformation_matrix(a, b, k):
        I = np.identity(2)
        k_k = np.outer(k, k)  # Outer product of the direction vector
        T = (1 / b) * (I + (b/a - 1) * k_k)  # Transformation matrix
        return T

    # Transform the first ellipse into a unit circle
    T1 = transformation_matrix(a1, b1, k1)

    # Step 3: Transform the second ellipse using the same transformation matrix
    # Ellipse matrix (representing the shape of the ellipse in quadratic form)
    def ellipse_matrix(a, b, k):
        I = np.identity(2)
        k_k = np.outer(k, k)
        return (1 / b**2) * (I + (b**2 / a**2 - 1) * k_k)

    A2 = ellipse_matrix(a2, b2, k2)  # Shape matrix for ellipse 2
    A2_transformed = np.dot(np.dot(np.linalg.inv(T1), A2), np.linalg.inv(T1))  # Transforming ellipse 2

    # Step 4: Calculate the quartic equation to solve for the distance in the transformed space
    # For simplicity, using the transformed semi-major and semi-minor axes
    eigenvalues, _ = np.linalg.eig(A2_transformed)  # Eigenvalues of the transformed matrix
    a2_prime = 1 / np.sqrt(np.min(eigenvalues))  # Transformed semi-major axis
    b2_prime = 1 / np.sqrt(np.max(eigenvalues))  # Transformed semi-minor axis
    
    # The distance between the unit circle and the transformed ellipse can be computed analytically:
    def transformed_distance(a_prime, b_prime):
        # Approximation of the closest distance in transformed space
        return 1 + a_prime  # Special case from the paper, for external tangency
    
    d_prime = transformed_distance(a2_prime, b2_prime)

    # Step 5: Apply inverse transformation to get the real-world distance
    # Calculate the angle between the major axis of ellipse 1 and the vector joining the centers
    angle_between = np.dot(k1, position_diff / np.linalg.norm(position_diff))
    eccentricity1 = np.sqrt(1 - (b1**2 / a1**2))  # Eccentricity of ellipse 1
    
    # Apply inverse transformation to find the real-world distance
    distance = d_prime / np.sqrt(1 - eccentricity1**2 * angle_between**2)
    
    return distance
    
    
# def calculate_closest_distance(agent1, agent2):
#     """
#     Calculate the closest distance between two ellipses with semi-major axes a1, a2,
#     semi-minor axes b1, b2, and orientation vectors k1, k2.

#     Parameters:
#         a1, b1: Semi-major and semi-minor axes of ellipse 1.
#         a2, b2: Semi-major and semi-minor axes of ellipse 2.
#         k1, k2: Orientation unit vectors for ellipses 1 and 2.

#     Returns:
#         The closest distance between the centers of the two ellipses.
#     """
#     a1, b1 = agent1.a, agent1.b
#     a2, b2 = agent2.a, agent2.b
#     k1, k2 = normalize(agent1.velocity), normalize(agent2.velocity)
    
#     # Step 1: Transformation matrix T to transform E1 into a unit circle
#     e1 = np.sqrt(1 - (b1**2 / a1**2))
#     e2 = np.sqrt(1 - (b2**2 / a2**2))
    
#     # Equation (4): T is a transformation matrix that scales the ellipse
#     def transformation_matrix(a, b, k):
#         I = np.identity(2)
#         k_k = np.outer(k, k)
#         T = (1 / b) * (I + (b/a - 1) * k_k)
#         return T
    
#     T1 = transformation_matrix(a1, b1, k1)
    
#     epsilon = 1e-10  # A small number to prevent division by zero

#     # Step 2: Transform the second ellipse
#     dot_product = np.dot(T1, k2)
#     norm_dot_product = np.linalg.norm(dot_product)

#     # Add epsilon to avoid division by zero
#     if norm_dot_product < epsilon:
#         norm_dot_product = epsilon

#     d_hat = dot_product / norm_dot_product
    
#     # Step 3: Apply transformed distances (Equation 36)
#     def distance_transformed(a2, b2, e1, d_hat):
#         return (1 / b1) * (1 / np.sqrt(1 - (e1**2) * (np.dot(d_hat, k1))**2))
    
#     d_transformed = distance_transformed(a2, b2, e1, d_hat)
    
#     # Inverse transformation to get the actual distance (Equation 35)
#     distance = d_transformed / np.linalg.norm(T1)
    
#     return distance