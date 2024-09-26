import numpy as np
from force import Force

class Agent:
    def __init__(self, position, velocity, mass=1.0, a_min = 1.0, b_min = 0.5, desired_walking_speed = 1.3, tau = 0.5, f = 3, ellipse = True):
        # 2D position and velocity (as numpy arrays for vector operations)
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.mass = mass  # Mass of the agent, useful for force calculations
        if b_min == 0.0:
            self.radius = a_min  # Radius, representing agent size
        else:
            self.a_min = a_min
            self.b_min = b_min
        self.ellipse = ellipse
        self.desired_walking_speed = desired_walking_speed

        self.tau = tau
        self.f = f
        ### internal variables ###
        self.driving_force = np.array([0.0, 0.0])
        self.repulsion_force = np.array([0.0, 0.0])
        self.total_force = self.driving_force + self.repulsion_force

        self.a = self.a_min + self.tau * np.linalg.norm(self.velocity)
        self.b = self.f * self.b_min - ((self.f-1) * self.b_min * np.linalg.norm(self.velocity) / self.desired_walking_speed)

        self.stopped = False

    
    def move(self, dt):
        self.total_force = self.driving_force + self.repulsion_force
        # print(f'total_force:{self.total_force}')
        # print(f'driving_force:{self.driving_force}')
        # print(f'repulsion force:{self.repulsion_force}')
        acceleration = self.total_force / self.mass
        self.velocity += acceleration * dt
        # Update the agent's position based on its velocity
        self.position += self.velocity * dt

        self.a = self.a_min + self.tau * np.linalg.norm(self.velocity)
        self.b = self.f * self.b_min - ((self.f-1) * self.b_min * np.linalg.norm(self.velocity) / self.desired_walking_speed)

        if self.position[0] > 50 and self.stopped == False:
            self.velocity = np.array([0.0, 0.0])
            self.stopped = True
        

        
    def reset(self):
        self.repulsion_force = np.array([0.0, 0.0])

        
    def __repr__(self):
        return f"Agent(pos={self.position}, vel={self.velocity})"