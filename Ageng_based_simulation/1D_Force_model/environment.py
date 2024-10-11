import numpy as np
class Environment:
    def __init__(self, width, height):
        self.width = width
        self.height = height  # Location offset for the bottleneck's center


        
        # Walls are only used if periodic is False
        self.walls = [
            {'equation': 'x = {}'.format(0),
                'x_range': (0, 0),
                'y_range': (0, self.height)},  # left-most wall

            {'equation': 'x = {}'.format(self.width),
                'x_range': (self.width, self.width),
                'y_range': (0, self.height)},  # right-most wall

            {'equation': 'y = {}'.format(0),
                'x_range': (0, self.width),
                'y_range': (0, 0)},  # bottom wall

            {'equation': 'y = {}'.format(self.height),
                'x_range': (0, self.width),
                'y_range': (self.height, self.height)}]  # top wall


    def apply_periodic_boundary(self, agent):
        """Apply periodic boundary condition for agents using modulo."""
        if self.periodic:
            x = agent.position
            if x > self.width:
                x = x % self.width
                agent.position = x
                return True
            else:
                return False
        else:
            return False
            
    def write_memory(self, agent, x_min, x_max, current_time):
        if agent.testing and not agent.tested and agent.memory['t_in'] == -1:
            agent.memory['t_in'] = current_time
            agent.memory['x_min'] = x_min
        
        elif not agent.testing and agent.tested and agent.memory['t_out'] == -1:
            agent.memory['t_out'] = current_time
            agent.memory['x_max'] = x_max

    def __repr__(self):
        return (f"Environment(width={self.width}, height={self.height}, ")