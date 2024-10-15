import numpy as np
from force import Force
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.animation import FFMpegWriter
import csv

class Force_Model:
    def __init__(self, environment, agents, parameters = (0.2, 1, 1, 1) ,tau = 0.5,  x_min = 0, x_max = 26):
        self.environment = environment
        self.agents = agents
        self.parameters = parameters
        self.mu = parameters[0]
        self.sigma = parameters[1]
        self.q = parameters[2]
        self.av_dash = parameters[3]
        self.tau = tau
        
        self.force = Force(self.agents, self.environment, tau = self.tau, mu = self.mu, sigma = self.sigma, q = self.q)
        
        self.current_step = 0
        self.x_min = x_min
        self.x_max = x_max


    def heun_scheme(self, dt):
        for i in range(len(self.agents)):
            if i == len(self.agents) - 1:
                j = 0
            else:
                j = i + 1
            self.force.algebraically_decaying_repulsive(i, j, stage2=False)
            self.agents[i].pseudo_move(dt)
            self.force.algebraically_decaying_repulsive(i, j, stage2=True)
            self.agents[i].move(dt)
            
        
    def new_force_scheme(self, dt):
        for i in range(len(self.agents)):
            if self.agents[i].velocity_dash < 0:
                j = i - 1
                
            else:
                j = i + 1
            if j == len(self.agents):
                j = 0
            elif j == 0:
                j = -1
            self.force.new_repulsive_force_heun(i, j, stage2=False)
            # self.agents[i].pseudo_move(dt)
            self.force.new_repulsive_force_heun(i, j, stage2=True)
            self.agents[i].move(dt)
            
            
    # def new_force_scheme(self, dt):
    #     """
    #     Computes the forces based on the closest neighbor interaction using Heun's method.
    #     """
    #     num_agents = len(self.agents)

    #     for i in range(num_agents):
    #         closest_j = None
    #         min_separation = float('inf')

    #         # Find the closest neighbor
    #         for j in range(num_agents):
    #             if i != j:
    #                 # Calculate separation between agents i and j
    #                 separation = abs(self.agents[j].x_dash - self.agents[i].x_dash)
                    
    #                 # Apply periodic boundary conditions
    #                 separation = min(separation, self.environment.width - separation)

    #                 # Find the agent with the minimum separation
    #                 if separation < min_separation:
    #                     min_separation = separation
    #                     closest_j = j

    #         # Once we have found the closest neighbor, we perform the force calculations
    #         if closest_j is not None:
    #             # Stage 1: Predictor step
    #             self.force.new_repulsive_force_heun(i, closest_j, stage2=False)
    #             self.agents[i].pseudo_move(dt)
                
    #             # Stage 2: Corrector step
    #             self.force.new_repulsive_force_heun(i, closest_j, stage2=True)
    #             self.agents[i].move(dt)
    
    def update(self, dt):
        """ Update the state of the model by one time step """
        self.current_step += 1
        flipped = False
        # print(self.current_step)
        if self.current_step % 10 == 0:
            velocities = np.array([agent.velocity_dash for agent in self.agents])
            velocity_std = np.std(velocities)
            if velocity_std > 5 or np.any(np.abs(velocities) > 10):
                return False
            
            # Append the standard deviation to a CSV file
            with open(r'C:\\Users\\Peter\\OneDrive - University of Edinburgh\\Desktop\\new_model_std.csv', mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([self.current_step, velocity_std, self.parameters])
            
            with open(r'C:\\Users\\Peter\\OneDrive - University of Edinburgh\\Desktop\\new_model_trajectory.csv', mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([self.current_step, self.agents[0].position, self.parameters])
                    
        # self.heun_scheme(dt)
        self.new_force_scheme(dt)
        
        return True
        

        
        # for agent in self.agents:
        #     agent.move(dt)  # Move the agent based on the updated velocity



    def animate(self, steps, dt=0.1, interval=100, output_filename="crowd_simulation.gif", show_forces=False, save_interval=100):
        """ Create an animation of the system over a given number of steps and save it as a GIF """
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.environment.width)
        ax.set_ylim(0, 2)  # Static y-limit for 1D visualization
        ax.set_aspect('equal')

        # Draw the walls and bottleneck
        try:
            for wall in self.environment.walls:
                ax.plot(wall['x_range'], [0, 0], 'k-', lw=2)  # Plot walls along y=1.0 for 1D
        except Exception as e:
            print(f"Error plotting walls: {e}")
            return

        # Initialize agent positions as unfilled circles (in 2D, but agents only move along x)
        try:
            agent_circles = [
                patches.Ellipse((agent.position, 1.0), 2*agent.a, 0.2, angle=0, color='b', fill=False, edgecolor='b')
                for agent in self.agents
            ]
            for circle in agent_circles:
                ax.add_patch(circle)
        except Exception as e:
            print(f"Error initializing agent circles: {e}")
            return

        forces = None

        # Create placeholders for force vectors if show_forces is True
        if show_forces:
            try:
                forces = ax.quiver(
                    [agent.position for agent in self.agents],  # X positions
                    [1.0 for _ in self.agents],  # Static Y position (1.0)
                    [agent.total_force for agent in self.agents],  # Force in X direction
                    [0 for _ in self.agents],  # No force in Y direction (1D sim)
                    angles='xy', scale_units='xy', scale=1, color='r', width=0.003
                )
            except Exception as e:
                print(f"Error initializing forces: {e}")
                return

        def init():
            """ Initialize the positions of agents """
            try:
                for agent, circle in zip(self.agents, agent_circles):
                    circle.center = (agent.position, 1.0)  # Set y=1.0 for display
            except Exception as e:
                print(f"Error during initialization: {e}")
            return agent_circles + ([forces] if forces else [])

        def update_animation(frame):
            """ Update the positions of agents for the animation """
            nonlocal forces
            try:
                self.update(dt)  # Update agent positions based on the model's time step

                # Remove agent circles if agents were removed
                while len(agent_circles) > len(self.agents):
                    old_circle = agent_circles.pop()
                    old_circle.remove()

                # Update existing circles' positions
                for agent, circle in zip(self.agents, agent_circles):
                    circle.center = (agent.position, 1.0)  # Set y=1.0 for display
                    circle.width = 2 * agent.a

                # Recreate force vectors if showing forces
                if show_forces:
                    if forces:
                        forces.remove()  # Remove old quiver plot
                    forces = ax.quiver(
                        [agent.position for agent in self.agents],  # X positions
                        [1.0 for _ in self.agents],  # Static Y positions (1.0)
                        [agent.total_force for agent in self.agents],  # Force in X direction
                        [0 for _ in self.agents],  # No Y direction (1D sim)
                        angles='xy', scale_units='xy', scale=1, color='r', width=0.003
                    )
            except Exception as e:
                print(f"Error during animation update: {e}")
                return []

            return agent_circles + ([forces] if show_forces else [])

        # Create a list of frames to save (only every `save_interval` frame)
        save_frames = range(0, steps, save_interval)

        # Set up the animation
        try:
            anim = FuncAnimation(
                fig, update_animation, frames=steps, init_func=init, blit=False, interval=interval
            )
        except Exception as e:
            print(f"Error setting up animation: {e}")
            return

        # Save only the selected frames (every `save_interval` frame)
        try:
            anim.save(output_filename, writer='pillow', fps=10)
        except Exception as e:
            print(f"Error saving animation: {e}")

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
            keep_going = self.update(dt)
            if not keep_going:
                print('dead by velocity')
                break
            # Log the state of the simulation at regular intervals
            if step % log_interval == 0:
                print(f"Step {step}:")
                agent = self.agents[0]
                print(f"  Agent {0}: Position: {agent.position}, Velocity: {agent.velocity}, Force: {agent.total_force}")
                # print(f" Test_agent: {agent.test}, agent_testing: {agent.testing}, agent_tested: {agent.tested}")
                # print(f" passed {self.agent_count_passed}")

            # Debug: If something specific is wrong, you could add more checks or logging here

        print("Simulation complete.")
