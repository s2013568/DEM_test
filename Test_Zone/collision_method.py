import numpy as np

class collisioncoord:
    def __init__(self, colliding_particle, iswall = False):
        """
        initialise the collision method called collision coordinates. see paper "DISCRETE PARTICLE SIMULATION OF BUBBLE AND SLUG FORMATION IN TWO DIMENSIONAL GAS FLUIDISED BED"

        colliding_particle is a tuple of two colliding particles
        iswall is a boolean that classify a wall collision or a particle collision
        """
        self.colliding_particle = colliding_particle
        self.iswall = False
    
def translate_velocity_to_collision_axes(p1, p2):
    # Step 1: Compute the vector from particle 1 to particle 2
    delta_pos = p2.position - p1.position
    distance = np.linalg.norm(delta_pos)
    
    # Step 2: Compute the unit vector for the y_col axis (center-to-center direction)
    if distance == 0:
        raise ValueError("Particles are at the same position; cannot compute collision axes.")
    y_col_axis = delta_pos / distance
    
    # Step 3: Compute the unit vector for the x_col axis (perpendicular to y_col axis)
    # The perpendicular direction can be computed using the 2D rotation matrix
    x_col_axis = np.array([-y_col_axis[1], y_col_axis[0]])
    
    # Step 4: Project the velocity of particle 1 onto the x_col and y_col axes
    v1_x_col = np.dot(p1.velocity, x_col_axis)
    v1_y_col = np.dot(p1.velocity, y_col_axis)
    
    # Step 5: Project the velocity of particle 2 onto the x_col and y_col axes
    v2_x_col = np.dot(p2.velocity, x_col_axis)
    v2_y_col = np.dot(p2.velocity, y_col_axis)
    
    # Return the velocities in the collision coordinate system
    return (v1_x_col, v1_y_col), (v2_x_col, v2_y_col)


    