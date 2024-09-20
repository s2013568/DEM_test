import numpy as np
import math

def translate_velocity_to_collision_axes(colliding_particle):
    p1, p2 = colliding_particle
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
    return ((v1_x_col, v1_y_col), (v2_x_col, v2_y_col))

def calculate_final_velocities(colliding_particle):
    """
    initialise the collision method called collision coordinates. see paper "DISCRETE PARTICLE SIMULATION OF BUBBLE AND SLUG FORMATION IN TWO DIMENSIONAL GAS FLUIDISED BED"
    this method assumes constant e and mu, and this applies to all particles.

    colliding_particle is a tuple of two colliding particles
    iswall is a boolean that classify a wall collision or a particle collision
    """

    ### deried variables
    collision_velocities= translate_velocity_to_collision_axes(colliding_particle)
    S0 = (collision_velocities[0][0] + (colliding_particle[0].ang_velocity * colliding_particle[0].radius)) - (collision_velocities[1][0] + (colliding_particle[1].ang_velocity * colliding_particle[1].radius))
    B1 = (1 / colliding_particle[0].mass) + (1 / colliding_particle[1].mass) + (colliding_particle[0].radius ** 2 / colliding_particle[0].moment_of_inertia) + (colliding_particle[1].radius ** 2 / colliding_particle[1].moment_of_inertia)
    C0 = collision_velocities[0][1] - collision_velocities[1][1]
    B2 = (1 / colliding_particle[0].mass) + (1 / colliding_particle[1].mass)

    PY = - (1 + colliding_particle[0].material_properties.get('restitution', 1.0)) * C0 / B2

    PXslide = - colliding_particle[0].material_properties.get('kinetic friction coeficient', 0) * math.copysign(1, S0) * PY
    PXstick = - S0 / B1

    if abs(S0) >= colliding_particle[0].material_properties.get('kinetic friction coeficient', 0) * B1 * PY:
        PX = PXslide
    else:
        PX = PXstick


    v_xa = PX / colliding_particle[0].mass + collision_velocities[0][0]
    v_ya = PY / colliding_particle[0].mass + collision_velocities[0][1]
    colliding_particle[0].ang_velocity = (PX * colliding_particle[0].radius / colliding_particle[0].moment_of_inertia) + colliding_particle[0].ang_velocity
    v_xb = - PX / colliding_particle[1].mass + collision_velocities[1][0]
    v_yb = - PY / colliding_particle[1].mass + collision_velocities[1][1]
    colliding_particle[1].ang_velocity = (- PX * colliding_particle[1].radius / colliding_particle[1].moment_of_inertia) + colliding_particle[1].ang_velocity

    print(v_xa, v_ya, v_xb, v_yb)

    return v_xa, v_ya, v_xb, v_yb


def translate_velocity_to_original_axes(v_xa, v_ya, v_xb, v_yb, p1, p2):
    # Step 1: Compute the vector from particle 1 to particle 2
    delta_pos = p2.position - p1.position
    distance = np.linalg.norm(delta_pos)
    
    # Step 2: Compute the unit vector for the y_col axis (center-to-center direction)
    if distance == 0:
        raise ValueError("Particles are at the same position; cannot compute collision axes.")
    y_col_axis = delta_pos / distance
    
    # Step 3: Compute the unit vector for the x_col axis (perpendicular to y_col axis)
    x_col_axis = np.array([-y_col_axis[1], y_col_axis[0]])
    
    # Step 4: Reconstruct the velocity of particle 1 in the original coordinate system
    p1.velocity = v_xa * x_col_axis + v_ya * y_col_axis
    print(p1.velocity)
    # Step 5: Reconstruct the velocity of particle 2 in the original coordinate system
    if p2.id != -1:
        p2.velocity = v_xb * x_col_axis + v_yb * y_col_axis
    