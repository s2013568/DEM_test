from particle_defintions import Particle2D, Box2D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import collision_method as cm
import matplotlib.cm as mcm
import matplotlib.colorbar as colorbar


class DEM2D:
    def __init__(self, particles, box, time_step=0.01, total_time=10.0, gravity = True):
        """
        Initialize the DEM simulation.
        
        :param particles: A list of Particle2D objects to simulate.
        :param box: A Box2D object representing the container.
        :param time_step: Time step for the simulation.
        :param total_time: Total simulation time.
        """
        self.particles = particles
        self.box = box
        self.time_step = time_step
        self.total_time = total_time
        self.current_time = 0.0
        self.history = []  # Store history of particle states for visualization if needed
        self.gravity = gravity

        ### internal variables
        self.collisions = []


    def run(self):
        """
        Runs the DEM simulation for the specified total time.
        """
        num_steps = int(self.total_time / self.time_step)
        for step in range(num_steps):
            self.current_time += self.time_step
            # print(f"Step {step+1}/{num_steps}, Time: {self.current_time}")
            
            ### Check if any collision have occured
            self.check_for_collision()
            ### if collision have occured, we resolve them pair by pair, and updating their respective velocity internally
            if self.collisions != []:
                for colliding_particle in self.collisions:
                    v_xa, v_ya, v_xb, v_yb, omega0, omega1 = cm.calculate_final_velocities(colliding_particle)
                    # print(v_xa, v_ya, v_xb, v_yb, omega0, omega1)
                    cm.translate_velocity_to_original_axes(v_xa, v_ya, v_xb, v_yb, colliding_particle[0], colliding_particle[1])

                    total_energy = 0
                    for i in range(len(self.particles)):
                        energy = 0.5 * self.particles[i].mass * np.linalg.norm(self.particles[i].velocity)**2 + 0.5 * self.particles[i].moment_of_inertia * self.particles[i].ang_velocity**2
                        total_energy += energy
                    print(f'Total energy = {total_energy}')
                ### clear buffer
                self.collisions = []

            
            ### now the collision is solved, the particles are then allowed to step into the next time step
            for particle in self.particles:
                self.update_particle(particle)
            
            # Store history (optional, for later visualization)
            self.store_history()


    def check_for_collision(self):
        for i in range(len(self.particles)):
            for j in range(i + 1, len(self.particles)):  # To avoid checking the same pair twice
                p1 = self.particles[i]
                p2 = self.particles[j]
                
                # Calculate the distance between the two particles
                distance = np.linalg.norm(p1.position - p2.position)
                
                # Check if the distance is less than or equal to the sum of their radii
                if distance <= (p1.radius + p2.radius):
                    self.collisions.append((p1, p2))

            p = self.particles[i]
            x, y = p.position  # Particle's position

            # Check if the particle is colliding with the left or right wall
            if x - p.radius <= 0:
                self.collisions.append((p, Particle2D(np.array([-2* p.radius, y]), np.array([0, 0]), 0, p.radius, np.inf, material_properties = {'restitution': 0.9 , 'kinetic friction coeficient': 0.6}, id=-1)))
            elif x + p.radius >= self.box.width:
                self.collisions.append((p, Particle2D(np.array([self.box.width +2* p.radius, y]), np.array([0, 0]), 0, p.radius, np.inf, material_properties = {'restitution': 0.9 , 'kinetic friction coeficient': 0.6}, id=-1)))
            # Check if the particle is colliding with the top or bottom wall
            if y - p.radius <= 0:
                self.collisions.append((p, Particle2D(np.array([x, -2* p.radius]), np.array([0, 0]), 0, p.radius, np.inf, material_properties = {'restitution': 0.9 , 'kinetic friction coeficient': 0.6}, id = -1)))  # 'y' indicates vertical wall collision
            elif y + p.radius >= self.box.height:
                self.collisions.append((p, Particle2D(np.array([x, self.box.height +2* p.radius]), np.array([0, 0]), 0, p.radius, np.inf, material_properties = {'restitution': 0.9 , 'kinetic friction coeficient': 0.6}, id = -1)))  # 'y' indicates vertical wall collision


    def update_particle(self, particle):
        """
        Update the particle's position, handle boundary collision.
        
        :param particle: A Particle2D object to update.
        """

        # Reset forces on the particle (if we had any forces acting on them)
        particle.reset_force(gravity = self.gravity)
        
        # Update acceleration based on current forces (forces are zero in this case)
        particle.update_acceleration()

        particle.update_velocity(self.time_step)

        # Update particle's position, orientation and velocity using integration
        particle.update_position(self.time_step)
        particle.update_orientation(self.time_step)

    def check_for_contact(self, particle):
        """
        Check for contact between particles (Placeholder).
        To be implemented for particle-particle collisions.
        
        :param particle: A Particle2D object.
        """
        pass  # Implement particle-particle collision logic here in the future

    def store_history(self):
        """
        Store the current state of all particles for visualization.
        """
        particle_states = [(p.position.copy(), p.velocity.copy(), p.ang_velocity) for p in self.particles]
        self.history.append(particle_states)

    def visualize(self, save_path=None, save_format="gif", max_frames=100):
        """
        Visualize the particle positions over time using the stored history,
        with an option to limit the total number of frames in the animation.
        """
        fig, ax = plt.subplots()

        # Define the maximum number of frames for the animation
        total_steps = len(self.history)

        # If the total number of steps exceeds max_frames, we sample the frames
        if total_steps > max_frames:
            frame_indices = np.linspace(0, total_steps - 1, max_frames, dtype=int)
        else:
            frame_indices = np.arange(total_steps)

        # Define the color map for angular momentum
        cmap = mcm.coolwarm
        norm = plt.Normalize(vmin=-3, vmax=3)  # Assuming angular momentum range from -3 to 3

        # Add a color bar to represent angular momentum
        sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])  # ScalarMappable needs a dummy array to work with color bar
        fig.colorbar(sm, ax=ax, label="Angular Momentum")

        # Plot the initial state of particles in the box
        def init():
            ax.clear()
            ax.set_xlim(0, self.box.width)
            ax.set_ylim(0, self.box.height)
            ax.set_aspect('equal', 'box')
            ax.grid(True)

        # Plot the particles at a specific time step
        
        def update(step_index):
            step = frame_indices[step_index]  # Get the actual step from the downsampled indices
            ax.clear()
            init() 

            particle_states = self.history[step]

            for (pos, _, ang) in particle_states:
                # Normalize the angle to the range [0, 1] for colormap
                norm_ang = norm(ang)
                
                # Use the colormap to map normalized angle to a color
                color = cmap(norm_ang)
                
                circle = plt.Circle(pos, radius=self.particles[0].radius, color=color)
                ax.add_patch(circle)
                
            plt.title(f"Time: {self.current_time:.2f}")

        # Animation using the downsampled frame indices
        self.anim = FuncAnimation(fig, update, frames=len(frame_indices), interval=50)

        # Save the animation if a path is provided
        if save_path:
            if save_format == "gif":
                self.anim.save(save_path, writer="pillow")
            elif save_format == "mp4":
                self.anim.save(save_path, writer="ffmpeg")
            print(f"Animation saved to {save_path}")

        plt.show()

    def reset(self):
        """
        Resets the simulation state, if needed.
        """
        self.current_time = 0.0
        self.history = []




