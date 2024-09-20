from particle_defintions import Particle2D, Box2D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import collision_method as cm


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
            print(f"Step {step+1}/{num_steps}, Time: {self.current_time}")
            
            ### Check if any collision have occured
            self.check_for_collision()
            ### if collision have occured, we resolve them pair by pair, and updating their respective velocity internally
            if self.collisions != []:
                for colliding_particle in self.collisions:
                    v_xa, v_ya, v_xb, v_yb = cm.calculate_final_velocities(colliding_particle)
                    cm.translate_velocity_to_original_axes(v_xa, v_ya, v_xb, v_yb, colliding_particle[0], colliding_particle[1])
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
                self.collisions.append((p, Particle2D(np.array([x - p.radius, y]), np.array([0, 0]), 0, p.radius, np.inf, material_properties = {'restitution': 1.0 , 'kinetic friction coeficient': 0}, id=-1)))
            elif x + p.radius >= self.box.width:
                self.collisions.append((p, Particle2D(np.array([x + p.radius, y]), np.array([0, 0]), 0, p.radius, np.inf, material_properties = {'restitution': 1.0 , 'kinetic friction coeficient': 0}, id=-1)))
            # Check if the particle is colliding with the top or bottom wall
            if y - p.radius <= 0:
                self.collisions.append((p, Particle2D(np.array([x, y - p.radius]), np.array([0, 0]), 0, p.radius, np.inf, material_properties = {'restitution': 1.0 , 'kinetic friction coeficient': 0}, id = -1)))  # 'y' indicates vertical wall collision
            elif y + p.radius >= self.box.height:
                self.collisions.append((p, Particle2D(np.array([x, y + p.radius]), np.array([0, 0]), 0, p.radius, np.inf, material_properties = {'restitution': 1.0 , 'kinetic friction coeficient': 0}, id = -1)))  # 'y' indicates vertical wall collision


    def update_particle(self, particle):
        """
        Update the particle's position, handle boundary collision.
        
        :param particle: A Particle2D object to update.
        """

        # Reset forces on the particle (if we had any forces acting on them)
        particle.reset_force(gravity = True)
        
        # Update acceleration based on current forces (forces are zero in this case)
        particle.update_acceleration()

        # Update particle's position and velocity using integration
        particle.update_position(self.time_step)

        # # Handle boundary collisions
        # particle.handle_wall_collision((self.box.width, self.box.height))


        # Placeholder for particle-particle interactions (collision detection)
        self.check_for_contact(particle)

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
        particle_states = [(p.position.copy(), p.velocity.copy()) for p in self.particles]
        self.history.append(particle_states)

    def visualize(self, save_path=None, save_format="gif"):
        """
        Visualize the particle positions over time using the stored history.
        """
        fig, ax = plt.subplots()

        # Plot the initial state of particles in the box
        def init():
            ax.clear()
            ax.set_xlim(0, self.box.width)
            ax.set_ylim(0, self.box.height)
            ax.set_aspect('equal', 'box')
            ax.grid(True)
            plt.title(f"Time: {self.current_time:.2f}")

        # Plot the particles at a specific time step
        def update(step):
            ax.clear()
            init()
            particle_states = self.history[step]
            for (pos, _) in particle_states:
                circle = plt.Circle(pos, radius=self.particles[0].radius, color='g')
                ax.add_patch(circle)

        # Animation using stored history

        self.anim = FuncAnimation(fig, update, frames=len(self.history), interval=50)


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
