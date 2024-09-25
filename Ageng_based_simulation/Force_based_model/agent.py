import numpy as np

class Agent:
    def __init__(self, position, velocity, mass=1.0, radius=0.5):
        # 2D position and velocity (as numpy arrays for vector operations)
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.mass = mass  # Mass of the agent, useful for force calculations
        self.radius = radius  # Radius, representing agent size
    
    def move(self, dt):
        # Update the agent's position based on its velocity
        self.position += self.velocity * dt
        
    def apply_force(self, force, dt):
        # Apply a force to update the velocity
        acceleration = force / self.mass
        self.velocity += acceleration * dt
        
    def __repr__(self):
        return f"Agent(pos={self.position}, vel={self.velocity})"