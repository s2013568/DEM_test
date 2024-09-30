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