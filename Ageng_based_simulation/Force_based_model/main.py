import numpy as np
from force import Force
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class GCFModel:
    def __init__(self, environment, agents, desired_walking_speed = 1.3, time_constant = 1.0):
        self.environment = environment
        self.agents = agents
        self.force = Force(self.agents, self.environment, desired_walking_speed, time_constant = time_constant)

    def calculate_forces(self):
        self.force.point_direction_method(vector = (10, 5))
        self.force.calculate_repulsive_forces()

    def update(self, dt):
        """ Update the state of the model by one time step """
        self.calculate_forces()  # Calculate forces for all agents
        agents_to_remove = []

        for agent in self.agents:
            agent.move(dt)  # Move the agent based on the updated velocity
            
            # Check if the agent hits a wall, and reverse velocity if needed
            if not self.environment.is_within_walls(agent):
                agent.velocity *= -1  # Bounce off the walls for simplicity
                
            if agent.position[0] >= 10:
                agents_to_remove.append(agent)
                
        self.agents = [agent for agent in self.agents if agent not in agents_to_remove]


    def animate(self, steps, dt=0.1, interval=100, output_filename="crowd_simulation.gif", show_forces=False):
        """ Create an animation of the system over a given number of steps and save it as a GIF """
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

        # Initialize agent positions as unfilled circles
        agent_circles = [plt.Circle(agent.position, agent.radius, color='b', fill=False, edgecolor='b') for agent in self.agents]
        for circle in agent_circles:
            ax.add_patch(circle)
                
        forces = None

        # Create placeholders for force vectors if show_forces is True
        if show_forces:
            forces = ax.quiver(
                [agent.position[0] for agent in self.agents],  # X positions
                [agent.position[1] for agent in self.agents],  # Y positions
                [agent.total_force[0] for agent in self.agents],  # Force in X direction
                [agent.total_force[1] for agent in self.agents],  # Force in Y direction
                angles='xy', scale_units='xy', scale=1, color='r', width=0.003)

        def init():
            """ Initialize the positions of agents """
            for agent, circle in zip(self.agents, agent_circles):
                circle.center = agent.position
            if show_forces:
                return agent_circles + ([forces] if forces else [])
            return agent_circles

        def update_animation(frame):
            """ Update the positions of agents for the animation """
            nonlocal forces
            self.update(dt)

            # Remove agent circles if agents were removed and rebuild list
            new_agent_circles = [plt.Circle(agent.position, agent.radius, color='b', fill=False, edgecolor='b') for agent in self.agents]
            
            # Remove old circles
            while len(agent_circles) > len(self.agents):
                old_circle = agent_circles.pop()
                old_circle.remove()  # Remove the circle from the plot

            # Update existing circles' positions
            for agent, circle in zip(self.agents, agent_circles):
                circle.center = agent.position

            # Add new circles if necessary
            if len(new_agent_circles) > len(agent_circles):
                for new_circle in new_agent_circles[len(agent_circles):]:
                    ax.add_patch(new_circle)
                agent_circles.extend(new_agent_circles[len(agent_circles):])

            # Recreate force vectors if showing forces
            if show_forces:
                if forces:
                    forces.remove()  # Remove old quiver plot
                # Create new quiver plot to match the current number of agents
                forces = ax.quiver(
                    [agent.position[0] for agent in self.agents],  # X positions
                    [agent.position[1] for agent in self.agents],  # Y positions
                    [agent.total_force[0] for agent in self.agents],  # Force in X direction
                    [agent.total_force[1] for agent in self.agents],  # Force in Y direction
                    angles='xy', scale_units='xy', scale=1, color='r', width=0.003)

            return agent_circles + ([forces] if show_forces else [])

        # Set up the animation
        anim = FuncAnimation(fig, update_animation, frames=steps,
                            init_func=init, blit=True, interval=interval)

        # Save the animation as a GIF at 10 frames per second
        anim.save(output_filename, writer='pillow', fps=10000)

        plt.show()
