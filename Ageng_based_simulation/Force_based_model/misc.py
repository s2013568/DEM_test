import numpy as np
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

def closest_distance_between_ellipses(agent1, agent2):
    separation_unit_vector = normalize(agent1.position - agent2.position)
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
    
    h = np.linalg.norm(agent1.position - agent2.position) - r1 - r2
    
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
    Calculate the closest distance between two ellipses with semi-major axes a1, a2,
    semi-minor axes b1, b2, and orientation vectors k1, k2.

    Parameters:
        a1, b1: Semi-major and semi-minor axes of ellipse 1.
        a2, b2: Semi-major and semi-minor axes of ellipse 2.
        k1, k2: Orientation unit vectors for ellipses 1 and 2.

    Returns:
        The closest distance between the centers of the two ellipses.
    """
    
    a1, b1 = agent1.a, agent1.b
    a2, b2 = agent2.a, agent2.b
    k1, k2 = normalize(agent1.velocity), normalize(agent2.velocity)
    
    # Step 1: Transformation matrix T to transform E1 into a unit circle
    e1 = np.sqrt(1 - (b1**2 / a1**2))
    e2 = np.sqrt(1 - (b2**2 / a2**2))
    
    # Equation (4): T is a transformation matrix that scales the ellipse
    def transformation_matrix(a, b, k):
        I = np.identity(2)
        k_k = np.outer(k, k)
        T = (1 / b) * (I + (b/a - 1) * k_k)
        return T
    
    T1 = transformation_matrix(a1, b1, k1)
    
    # Step 2: Transform the second ellipse
    d_hat = np.dot(T1, k2) / np.linalg.norm(np.dot(T1, k2))
    
    # Step 3: Apply transformed distances (Equation 36)
    def distance_transformed(a2, b2, e1, d_hat):
        return (1 / b1) * (1 / np.sqrt(1 - e1**2 * (np.dot(d_hat, k1))**2))
    
    d_transformed = distance_transformed(a2, b2, e1, d_hat)
    
    # Inverse transformation to get the actual distance (Equation 35)
    distance = d_transformed / np.linalg.norm(T1)
    
    return distance