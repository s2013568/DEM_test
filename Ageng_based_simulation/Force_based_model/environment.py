class Environment:
    def __init__(self, width, height, bottleneck_width, bottleneck_height):
        self.width = width
        self.height = height
        self.bottleneck_width = bottleneck_width
        self.bottleneck_height = bottleneck_height

        # Define the left and right regions and bottleneck
        self.left_region = {'x_min': 0, 'x_max': (width - bottleneck_width) / 2, 'y_min': 0, 'y_max': height}
        self.right_region = {'x_min': (width + bottleneck_width) / 2, 'x_max': width, 'y_min': 0, 'y_max': height}
        self.bottleneck = {'x_min': (width - bottleneck_width) / 2, 
                           'x_max': (width + bottleneck_width) / 2,
                           'y_min': (height - bottleneck_height) / 2,
                           'y_max': (height + bottleneck_height) / 2}

    def is_in_bottleneck(self, agent):
        """Check if an agent is within the bottleneck region."""
        x, y = agent.position
        return (self.bottleneck['x_min'] <= x <= self.bottleneck['x_max'] and
                self.bottleneck['y_min'] <= y <= self.bottleneck['y_max'])

    def is_within_walls(self, agent):
        """Check if the agent is within the walls of the environment."""
        x, y = agent.position
        # Check left region
        if x <= self.left_region['x_max'] and y <= self.left_region['y_max']:
            return True
        # Check right region
        if x >= self.right_region['x_min'] and y <= self.right_region['y_max']:
            return True
        # Check if in bottleneck
        return self.is_in_bottleneck(agent)

    def __repr__(self):
        return (f"Environment(width={self.width}, height={self.height}, "
                f"bottleneck_width={self.bottleneck_width}, bottleneck_height={self.bottleneck_height})")
