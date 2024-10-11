import numpy as np
from force import Force
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.animation import FFMpegWriter

class Force_Model:
    def __init__(self, environment, agents, parameters = (0.2, 1, 1, 1),tau = 0.5,  x_min = 0, x_max = 26):
        self.environment = environment
        self.agents = agents
        self.mu = parameters[0]
        self.sigma = parameters[1]
        self.q = parameters[2]
        self.av_dash = parameters[3]
        self.tau = tau
        
        self.force = Force(self.agents, self.environment, tau = self.tau, mu = self.mu, sigma = self.sigma, q = self.q)
        
        self.current_step = 0
        self.x_min = x_min
        self.x_max = x_max


    def heun_scheme(self):
        for i in range(len(self.agents)):
            if i == len(self.agents) - 1:
                j = 0
            else:
                j = i + 1
            self.force.algebraically_decaying_repulsive(i, j, stage2=False)
            self.agents[i].pseudo_move()
            self.force.algebraically_decaying_repulsive(i, j, stage2=True)
            self.agents[i].move()
            
        
    
    
    def update(self, dt):
        """ Update the state of the model by one time step """
        self.current_step += 1
        flipped = False
        # print(self.current_step)
        
        self.heun_scheme()
        
        # for agent in self.agents:
        #     agent.move(dt)  # Move the agent based on the updated velocity



    def animate(self, steps, dt=0.1, interval=100, output_filename="crowd_simulation.gif", show_forces=False):
        """ Create an animation of the system over a given number of steps and save it as a GIF """
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.environment.width)
        ax.set_ylim(0, self.environment.height)
        ax.set_aspect('equal')

        # Draw the walls and bottleneck
        for wall in self.environment.walls:
            ax.plot(wall['x_range'], wall['y_range'], 'k-', lw=2)  # 'k-' means black solid line

        # Initialize agent positions as unfilled circles
        # agent_circles = [plt.Circle(agent.position, agent.radius, color='b', fill=False, edgecolor='b') for agent in self.agents]
        agent_circles = [patches.Ellipse((agent.position, 2), 2*agent.a, 2*agent.b,0, color='b', fill=False, edgecolor='b') for agent in self.agents]
        for circle in agent_circles:
            ax.add_patch(circle)
                
        forces = None

        # Create placeholders for force vectors if show_forces is True
        if show_forces:
            forces = ax.quiver(
                [agent.position for agent in self.agents],  # X positions
                [2 for agent in self.agents],  # Y positions
                [agent.total_force for agent in self.agents],  # Force in X direction
                [0 for agent in self.agents],  # Force in Y direction
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
            # new_agent_circles = [plt.Circle(agent.position, agent.radius, color='b', fill=False, edgecolor='b') for agent in self.agents]
            new_agent_circles = [patches.Ellipse(agent.position, 2*agent.a, 2*agent.b, angle=0, color='b', fill=False, edgecolor='b') for agent in self.agents]
            
            # Remove old circles
            while len(agent_circles) > len(self.agents):
                old_circle = agent_circles.pop()
                old_circle.remove()  # Remove the circle from the plot

            # Update existing circles' positions
            for agent, circle in zip(self.agents, agent_circles):
                circle.center = agent.position
                circle.angle = 0
                circle.width = 2 * agent.a
                circle.height = 2 * agent.b

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
                    [agent.position for agent in self.agents],  # X positions
                    [2 for agent in self.agents],  # Y positions
                    [agent.total_force for agent in self.agents],  # Force in X direction
                    [0 for agent in self.agents],  # Force in Y direction
                    angles='xy', scale_units='xy', scale=1, color='r', width=0.003)

            return agent_circles + ([forces] if show_forces else [])

        # Set up the animation
        anim = FuncAnimation(fig, update_animation, frames=steps,
                            init_func=init, blit=True, interval=interval)

        # Save the animation as a GIF at 10 frames per second
        anim.save(output_filename, writer='pillow', fps=10)
        # anim.save(output_filename, writer='ffmpeg', fps=10000, codec='libx264')

        # writer = FFMpegWriter(fps=100)

        # # Save the animation using FFMpeg writer
        # anim.save(output_filename, writer=writer)
        

        plt.show()



    def run_simulation(self, steps, dt=0.1, log_interval=10, verbose=True):
        """
        Run the simulation for a given number of steps without animation.
        Useful for debugging by logging agent positions, velocities, and forces.

        Parameters:
        - steps: Number of steps to run the simulation.
        - dt: Time step for each update.
        - log_interval: How often to log the simulation state (in number of steps).
        - verbose: If True, print agent states to the console.
        """
        for step in range(steps):
            # Update the positions and forces of agents
            self.update(dt)

            # Log the state of the simulation at regular intervals
            if step % log_interval == 0:
                if verbose:
                    print(f"Step {step}:")
                    agent = self.agents[0]
                    print(f"  Agent {0}: Position: {agent.position}, Velocity: {agent.velocity}, Force: {agent.total_force}")
                    print(f" Test_agent: {agent.test}, agent_testing: {agent.testing}, agent_tested: {agent.tested}")
                    print(f" passed {self.agent_count_passed}")

            # Debug: If something specific is wrong, you could add more checks or logging here

        print("Simulation complete.")
