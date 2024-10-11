import numpy as np
from force import Force

class Agent:
    def __init__(self, position, velocity, a0 =1, b0 = 1, v0 = 1.2, av = 0.5, tau = 0.5, test = False):
        # 1D position and velocity (as numpy arrays for vector operations)
        self.position = position
        self.velocity = velocity
        self.a0 = a0
        self.b0 = b0
        self.tau = tau
        self.v0 = v0

        self.av = av

        ### internal variables ###
        self.driving_force = 0
        self.repulsion_force = 0
        self.total_force = 0
        self.a = self.a0 + self.av * self.velocity
        
        
        self.av_dash = self.av / self.tau
        self.x_dash = self.position / self.a0
        self.v0_dash = self.v0 * self.tau / self.a0
        self.velocity_dash = velocity * self.tau / self.a0
        self.a_dash = 1 + (self.av / self.tau) * self.velocity_dash
        
        self.width_dash = 200 / self.a0
        
        self.k1 = 0
        self.buffer = {
            'position': self.position,
            'velocity': self.velocity,
            'x_dash': self.x_dash,
            'velocity_dash': self.velocity_dash,
            'a': self.a,
            'a_dash': self.a_dash
        }
        
        
    def pseudo_move(self, dt):
        # Create a buffer to store temporary values without altering the original state
        buffer = {
            'position': self.position,
            'velocity': self.velocity,
            'x_dash': self.x_dash,
            'velocity_dash': self.velocity_dash,
            'a': self.a,
            'a_dash': self.a_dash
        }
        
        # Simulate the movement in the buffer
        acceleration_dash = self.k1
        buffer['velocity_dash'] += acceleration_dash * dt
        buffer['velocity'] = buffer['velocity_dash'] * self.a0 / self.tau
        buffer['x_dash'] += buffer['velocity_dash'] * dt
        x = buffer['x_dash']
        if x > self.width_dash:
            x = x % self.width_dash
            buffer['x_dash'] = x
        buffer['position'] = buffer['x_dash'] * self.a0




        # Update a and a_dash in the buffer
        buffer['a_dash'] = 1 + (self.av / self.tau) * buffer['velocity_dash']
        buffer['a'] = self.a0 + self.av * buffer['velocity']
        
        self.buffer = buffer
    
    def move(self, dt):
        
        
        
        acceleration_dash = self.total_force
        self.velocity_dash += acceleration_dash * dt
        self.velocity = self.velocity_dash * self.a0 / self.tau
        # Update the agent's position based on its velocity
        self.x_dash += self.velocity_dash * dt
        x = self.x_dash
        if x > self.width_dash:
            x = x % self.width_dash
            self.x_dash = x
        
        self.position = self.x_dash * self.a0

        self.a_dash = 1 + (self.av / self.tau) * self.velocity_dash
        # print(f'velocity_dash {self.velocity_dash}')
        self.a = self.a0 + self.av * self.velocity

        # if self.position[0] > 50 and self.stopped == False:
        #     self.velocity = np.array([0.0, 0.0])
        #     self.stopped = True
        

        
    def reset(self):
        self.repulsion_force = np.array([0.0, 0.0])
        
    def reset_wall(self):
        self.wall_force = np.array([0.0, 0.0])
        
    def add_force(self):
        self.total_force = self.repulsion_force + self.driving_force

        
    def __repr__(self):
        return f"Agent(pos={self.position}, vel={self.velocity})"