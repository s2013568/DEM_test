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
    
    return np.linalg.norm(agent1.position - agent2.position) - r1 - r2

def calculate_angle(velocity):
    # Compute the angle in degrees from the velocity vector
    return np.degrees(np.arctan2(velocity[1], velocity[0]))