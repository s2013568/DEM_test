import numpy as np
class Environment:
    def __init__(self, width, height, bottleneck_width, bottleneck_height, location=(0, 0), periodic=False):
        self.width = width
        self.height = height
        self.bottleneck_width = bottleneck_width
        self.bottleneck_height = bottleneck_height
        self.location = location  # Location offset for the bottleneck's center
        self.periodic = periodic  # Periodic boundary condition flag

        # Calculate bottleneck's center position based on the location offset
        center_x = width / 2 + location[0]
        center_y = height / 2 + location[1]

        # Define the left and right regions and bottleneck
        self.left_region = {'x_min': 0, 'x_max': center_x - bottleneck_width / 2, 'y_min': 0, 'y_max': height}
        self.right_region = {'x_min': center_x + bottleneck_width / 2, 'x_max': width, 'y_min': 0, 'y_max': height}
        self.bottleneck = {'x_min': center_x - bottleneck_width / 2, 
                           'x_max': center_x + bottleneck_width / 2,
                           'y_min': center_y - bottleneck_height / 2,
                           'y_max': center_y + bottleneck_height / 2}

        self.ins_density = []
        
        # Walls are only used if periodic is False
        if not self.periodic:
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

            if bottleneck_height != 0 and bottleneck_width != 0:
                self.walls.extend([
                    # Left region walls (vertical, x = constant)
                    {'equation': 'x = {}'.format(self.left_region['x_max']),
                     'x_range': (self.left_region['x_max'], self.left_region['x_max']),
                     'y_range': (0, self.bottleneck['y_min'])},  # Lower vertical wall

                    {'equation': 'x = {}'.format(self.left_region['x_max']),
                     'x_range': (self.left_region['x_max'], self.left_region['x_max']),
                     'y_range': (self.bottleneck['y_max'], self.height)},  # Upper vertical wall

                    # Right region walls (vertical, x = constant)
                    {'equation': 'x = {}'.format(self.right_region['x_min']),
                     'x_range': (self.right_region['x_min'], self.right_region['x_min']),
                     'y_range': (0, self.bottleneck['y_min'])},  # Lower vertical wall

                    {'equation': 'x = {}'.format(self.right_region['x_min']),
                     'x_range': (self.right_region['x_min'], self.right_region['x_min']),
                     'y_range': (self.bottleneck['y_max'], self.height)},  # Upper vertical wall

                    # Bottleneck walls (horizontal, y = constant)
                    {'equation': 'y = {}'.format(self.bottleneck['y_max']),
                     'x_range': (self.bottleneck['x_min'], self.bottleneck['x_max']),
                     'y_range': (self.bottleneck['y_max'], self.bottleneck['y_max'])},  # Top bottleneck wall

                    {'equation': 'y = {}'.format(self.bottleneck['y_min']),
                     'x_range': (self.bottleneck['x_min'], self.bottleneck['x_max']),
                     'y_range': (self.bottleneck['y_min'], self.bottleneck['y_min'])},  # Bottom bottleneck wall
                ])

    def is_in_bottleneck(self, agent):
        """Check if an agent is within the bottleneck region."""
        x, y = agent.position
        return (self.bottleneck['x_min'] <= x <= self.bottleneck['x_max'] and
                self.bottleneck['y_min'] <= y <= self.bottleneck['y_max'])

    def is_within_walls(self, agent):
        """Check if the agent is within the walls of the environment."""
        if self.periodic:
            # If periodic, there's no concept of walls
            return True
        
        x, y = agent.position
        # Check left region
        if x <= self.left_region['x_max'] and y <= self.left_region['y_max']:
            return True
        # Check right region
        if x >= self.right_region['x_min'] and y <= self.right_region['y_max']:
            return True
        # Check if in bottleneck
        return self.is_in_bottleneck(agent)

    def apply_periodic_boundary(self, agent):
        """Apply periodic boundary condition for agents using modulo."""
        if self.periodic:
            x, y = agent.position
            if x > self.width or y > self.height:
                x = x % self.width
                y = y % self.height
                agent.position = np.array([x, y])
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
        return (f"Environment(width={self.width}, height={self.height}, "
                f"bottleneck_width={self.bottleneck_width}, bottleneck_height={self.bottleneck_height}, "
                f"periodic={self.periodic})")
