import numpy as np
from force import Force

class Agent:
    def __init__(self, position, velocity, mass=1.0, radius=0.5, desired_walking_speed = 1.3):
        # 2D position and velocity (as numpy arrays for vector operations)
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.mass = mass  # Mass of the agent, useful for force calculations
        self.radius = radius  # Radius, representing agent size
        self.desired_walking_speed = desired_walking_speed
        
        ### internal variables ###
        self.driving_force = np.array([0.0, 0.0])
        self.repulsion_force = np.array([0.0, 0.0])
        self.total_force = self.driving_force + self.repulsion_force

    
    def move(self, dt):
        self.total_force = self.driving_force + self.repulsion_force
        # print(f'total_force:{self.total_force}')
        # print(f'driving_force:{self.driving_force}')
        # print(f'repulsion force:{self.repulsion_force}')
        acceleration = self.total_force / self.mass
        self.velocity += acceleration * dt
        # Update the agent's position based on its velocity
        self.position += self.velocity * dt
        

        
    def reset(self):
        self.repulsion_force = np.array([0.0, 0.0])

        
    def __repr__(self):
        return f"Agent(pos={self.position}, vel={self.velocity})"