import numpy as np

class Agent:
    def __init__(self, position, velocity, a0=1, b0=1, v0=1.2, av=0.5, tau=0.5, test=False):
        # 1D position and velocity (as numpy arrays for vector operations)
        self.position = position
        self.velocity = velocity
        self.a0 = a0  # Characteristic length scale
        self.b0 = b0  # Characteristic width scale
        self.tau = tau  # Characteristic time scale
        self.v0 = v0  # Desired velocity
        self.av = av  # Acceleration variable
        
        ### internal variables ###
        self.driving_force = 0
        self.repulsion_force = 0
        self.total_force = 0
        
        # Dimensional acceleration based on velocity
        self.a = self.a0 + self.av * self.velocity
        
        self.k1 = 0
        # Width of the system (now dimensional)
        self.width = 200
        
        # Buffers for storing the current state for potential updates without directly affecting the agent
        self.buffer = {
            'position': self.position,
            'velocity': self.velocity,
            'a': self.a
        }

    def pseudo_move(self, dt):
        # Create a buffer to store temporary values without altering the original state
        buffer = {
            'position': self.position,
            'velocity': self.velocity,
            'a': self.a
        }
        
        # Simulate the movement in the buffer
        acceleration = self.k1
        buffer['velocity'] += acceleration * dt
        buffer['position'] += buffer['velocity'] * dt
        
        # Handle periodic boundary conditions
        x = buffer['position']
        if x > self.width:
            x = x % self.width
            buffer['position'] = x
        elif x < 0:
            x = self.width + x
            buffer['position'] = x

        # Update the acceleration in the buffer
        buffer['a'] = self.a0 + self.av * buffer['velocity']
        
        # Store the buffer for later use
        self.buffer = buffer

    def move(self, dt):
        # Calculate acceleration based on total force
        acceleration = self.total_force
        self.velocity += acceleration * dt
        
        # Update the agent's position based on its velocity
        self.position += self.velocity * dt
        
        # Handle periodic boundary conditions
        if self.position > self.width:
            self.position = self.position % self.width
        elif self.position < 0:
            self.position = self.width + self.position
        
        # Update the acceleration based on the new velocity
        self.a = self.a0 + self.av * self.velocity

