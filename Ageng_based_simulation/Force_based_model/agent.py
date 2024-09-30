import numpy as np
from force import Force

class Agent:
    def __init__(self, position, velocity, mass=1.0, a_min = 0.18, b_min = 0.5, desired_walking_speed = 1.3, tau = 0.53, f = 3, ellipse = True, test = False):
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
        self.test = test
        ### internal variables ###
        self.driving_force = np.array([0.0, 0.0])
        self.repulsion_force = np.array([0.0, 0.0])
        self.total_force = self.driving_force + self.repulsion_force

        self.a = self.a_min + self.tau * np.linalg.norm(self.velocity)
        self.b = self.f * self.b_min - ((self.f-1) * self.b_min * np.linalg.norm(self.velocity) / self.desired_walking_speed)

        self.stopped = False
        
        self.memory = {'t_in' : -1, 
                       't_out' : -1,
                       'x_in' : 12,
                       'x_out' : 14}

    
    def move(self, dt):
        self.total_force = self.driving_force + self.repulsion_force
        # print(self.repulsion_force)
        # print(self.driving_force)
        # print(self.total_force)
        # print(f'total_force:{self.total_force}')
        # print(f'driving_force:{self.driving_force}')
        # print(f'repulsion force:{self.repulsion_force}')
        acceleration = self.total_force / self.mass
        self.velocity += acceleration * dt
        # Update the agent's position based on its velocity
        self.position += self.velocity * dt

        self.a = self.a_min + self.tau * np.linalg.norm(self.velocity)
        self.b = self.f * self.b_min - ((self.f-1) * self.b_min * np.linalg.norm(self.velocity) / self.desired_walking_speed)

        # if self.position[0] > 50 and self.stopped == False:
        #     self.velocity = np.array([0.0, 0.0])
        #     self.stopped = True
        

        
    def reset(self):
        self.repulsion_force = np.array([0.0, 0.0])
        
    def add_force(self):
        self.total_force = self.repulsion_force + self.driving_force

        
    def __repr__(self):
        return f"Agent(pos={self.position}, vel={self.velocity})"