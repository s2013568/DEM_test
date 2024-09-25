import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class GCFModel:
    def __init__(self, environment, agents):
        self.environment = environment
        self.agents = agents

    def calculate_forces(self):
        """ Placeholder method for force calculation. """
        for agent in self.agents:
            force = np.array([0.0, 0.0])  # Placeholder force
            agent.apply_force(force, dt=1)

    def update(self, dt):
        """ Update the state of the model by one time step """
        self.calculate_forces()  # Calculate forces for all agents
        for agent in self.agents:
            agent.move(dt)  # Move the agent based on the updated velocity
            
            # Check if the agent hits a wall, and reverse velocity if needed
            if not self.environment.is_within_walls(agent):
                agent.velocity *= -1  # Bounce off the walls for simplicity

    def animate(self, steps, dt=0.1, interval=100):
        """ Create an animation of the system over a given number of steps """
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.environment.width)
        ax.set_ylim(0, self.environment.height)
        ax.set_aspect('equal')

        # Draw the walls and bottleneck
        left_region = self.environment.left_region
        right_region = self.environment.right_region
        bottleneck = self.environment.bottleneck
        
        # Left wall
        ax.plot([left_region['x_max'], left_region['x_max']], [0, bottleneck['y_min']], 'k-')
        ax.plot([left_region['x_max'], left_region['x_max']], [bottleneck['y_max'], self.environment.height], 'k-')
        # Right wall
        ax.plot([right_region['x_min'], right_region['x_min']], [0, bottleneck['y_min']], 'k-')
        ax.plot([right_region['x_min'], right_region['x_min']], [bottleneck['y_max'], self.environment.height], 'k-')
        # Bottleneck walls (top and bottom)
        ax.plot([bottleneck['x_min'], bottleneck['x_max']], [bottleneck['y_max'], bottleneck['y_max']], 'k-')
        ax.plot([bottleneck['x_min'], bottleneck['x_max']], [bottleneck['y_min'], bottleneck['y_min']], 'k-')

        agent_circles = [plt.Circle(agent.position, agent.radius, color='b', fill=False, edgecolor='b') for agent in self.agents]
        for circle in agent_circles:
            ax.add_patch(circle)

        def init():
            """ Initialize the positions of agents """
            for agent, circle in zip(self.agents, agent_circles):
                circle.center = agent.position
            return agent_circles

        def update_animation(frame):
            """ Update the positions of agents for the animation """
            self.update(dt)
            for agent, circle in zip(self.agents, agent_circles):
                circle.center = agent.position
            return agent_circles

        anim = FuncAnimation(fig, update_animation, frames=steps,
                             init_func=init, blit=True, interval=interval)
        
        plt.show()