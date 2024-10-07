import numpy as np
from force import Force
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import misc
from matplotlib.animation import FFMpegWriter
import measurement

class GCFModel:
    def __init__(self, environment, agents, time_constant = 0.5, ellipse = True, eta = 1.0, x_min = 0, x_max = 26):
        self.environment = environment
        self.agents = agents
        self.time_constant = time_constant
        
        self.force = Force(self.agents, self.environment, time_constant = time_constant)
        self.ellipse = ellipse
        self.eta = eta
        
        self.current_step = 0
        self.x_min = x_min
        self.x_max = x_max
        self.environment.ins_density = []
        
        self.agent_count_passed = 0
        
        self.recorded_step_of_passing = []

    def calculate_forces(self):
        self.force = Force(self.agents, self.environment, time_constant = self.time_constant)
        # self.force.point_direction_method(line_points=((50, 0), (50, 3)))
        self.force.strat_1()
        # self.force.strat_2()
        self.force.calculate_repulsive_forces(eta = self.eta)
        self.force.calculate_wall_force()

    def update(self, dt):
        """ Update the state of the model by one time step """
        self.calculate_forces()  # Calculate forces for all agents
        self.current_step += 1
        agents_to_remove = []
        flipped = False
        print(self.current_step)
        for agent in reversed(self.agents):
            agent.move(dt)  # Move the agent based on the updated velocity
            
            
            # Check if the agent hits a wall, and reverse velocity if needed
            if not self.environment.is_within_walls(agent):
                agent.velocity *= -1  # Bounce off the walls for simplicity
                
            if self.environment.periodic:
                flipped = self.environment.apply_periodic_boundary(agent)
                if agent.test:
                    if self.current_step > 10000 and flipped and not agent.tested and not agent.testing:
                        agent.testing = True
                    elif self.current_step > 10000 and flipped and agent.testing:
                        agent.testing = False
                        agent.tested = True
            if agent.position[0] >= 26.4:
                agents_to_remove.append(agent)
                self.agent_count_passed += 1
                
                
            if agent.test:
                self.environment.write_memory(agent, self.x_min, self.x_max, self.current_step)

                if agent.testing and not agent.tested:
                    self.environment.ins_density.append(measurement.find_inst_time_density(agent, self.x_min, self.x_max, self.agents))
                
                
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
        # agent_circles = [plt.Circle(agent.position, agent.radius, color='b', fill=False, edgecolor='b') for agent in self.agents]
        agent_circles = [patches.Ellipse(agent.position, 2*agent.a, 2*agent.b,angle=misc.calculate_angle(agent.velocity), color='b', fill=False, edgecolor='b') for agent in self.agents]
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
            # new_agent_circles = [plt.Circle(agent.position, agent.radius, color='b', fill=False, edgecolor='b') for agent in self.agents]
            new_agent_circles = [patches.Ellipse(agent.position, 2*agent.a, 2*agent.b, angle=misc.calculate_angle(agent.velocity), color='b', fill=False, edgecolor='b') for agent in self.agents]
            
            # Remove old circles
            while len(agent_circles) > len(self.agents):
                old_circle = agent_circles.pop()
                old_circle.remove()  # Remove the circle from the plot

            # Update existing circles' positions
            for agent, circle in zip(self.agents, agent_circles):
                circle.center = agent.position
                circle.angle = misc.calculate_angle(agent.velocity)
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
        anim.save(output_filename, writer='pillow', fps=50000000)
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
            if self.agent_count_passed >= 50:
                self.recorded_step_of_passing.append(self.current_step)
                break

            # Log the state of the simulation at regular intervals
            if step % log_interval == 0:
                if verbose:
                    print(f"Step {step}:")
                    agent = self.agents[0]
                    print(f"  Agent {0}: Position: {agent.position}, Velocity: {agent.velocity}, Force: {agent.total_force}")
                    print(f" Test_agent: {agent.test}, agent_testing: {agent.testing}, agent_tested: {agent.tested}")

            # Debug: If something specific is wrong, you could add more checks or logging here

        print("Simulation complete.")
