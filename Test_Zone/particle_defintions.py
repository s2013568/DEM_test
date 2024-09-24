import numpy as np
import matplotlib.pyplot as plt


class Particle2D:
    def __init__(self, position, velocity, ang_velocity, orientation, radius, mass, material_properties = {'restitution': 1.0 , 'kinetic friction coeficient': 0}, id=None):
        """
        Initialize a 2D hard disk particle for DEM simulation.
        
        :param position: A 2D vector (list or numpy array) representing the particle's position.
        :param velocity: A 2D vector (list or numpy array) representing the particle's velocity.
        :param radius: Radius of the particle (scalar).
        :param mass: Mass of the particle (scalar).
        :param material_properties: Dictionary of material properties (e.g., {'restitution': 0.9}).
        :param id: (Optional) Unique identifier for the particle.   
        """
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.ang_velocity = ang_velocity
        self.orientation = orientation
        self.radius = radius
        self.mass = mass
        self.material_properties = material_properties
        self.id = id

        # Force and acceleration properties for 2D
        self.force = np.array([0.0, 0.0])  # Force acting on the particle
        self.acceleration = np.array([0.0, 0.0])  # Acceleration of the particle
        # magnitude = np.linalg.norm(self.velocity)
        # self.unit_direction = self.velocity / magnitude

        ### derived constants
        self.moment_of_inertia = (1/2) * self.mass * (self.radius ** 2)

    def update_acceleration(self):
        """
        Updates the particle's acceleration based on the current force and mass.
        """
        self.acceleration = self.force / self.mass

    def update_velocity(self, dt):
        self.velocity += self.acceleration * dt

    def update_position(self, dt):
        """
        Updates the particle's position and velocity using simple integration.
        
        :param dt: Time step for the simulation.
        """
        self.position += self.velocity * dt

    def update_orientation(self, dt):
        """
        Updates the particle's position and velocity using simple integration.
        
        :param dt: Time step for the simulation.
        """
        self.orientation += self.ang_velocity * dt

    def apply_force(self, force):
        """
        Adds an external force to the particle.
        
        :param force: A 2D vector representing the external force to be applied.
        """
        self.force += np.array(force)

    def reset_force(self, gravity):
        """
        Resets the force acting on the particle to zero. Call this at the beginning of each time step.
        """
        if not gravity:
            self.force = np.array([0.0, 0.0])
        else:
            self.force = np.array([0.0, -9.81 * self.mass])

    # def handle_wall_collision(self, box_size):
    #     """
    #     Handles collisions with the walls of a rectangular box.
        
    #     :param box_size: A tuple (width, height) defining the size of the box.
    #     """
    #     width, height = box_size
    #     # Collision with left or right wall
    #     if self.position[0] - self.radius < 0 or self.position[0] + self.radius > width:
    #         self.velocity[0] *= -self.material_properties.get('restitution', 1.0)
    #     # Collision with bottom or top wall
    #     if self.position[1] - self.radius < 0 or self.position[1] + self.radius > height:
    #         self.velocity[1] *= -self.material_properties.get('restitution', 1.0)

    def __repr__(self):
        """
        Custom string representation of the particle for easier debugging and logging.
        """
        return (f"Particle2D(id={self.id}, position={self.position}, velocity={self.velocity}, "
                f"radius={self.radius}, mass={self.mass}, material_properties={self.material_properties})")



class Box2D:
    def __init__(self, width=10, height=None):
        """
        Initialize a 2D box for containing particles.
        
        :param width: Width of the box (default is 10). If height is not provided, this will be used as the height as well (square box).
        :param height: Height of the box (optional, if not provided, defaults to width).
        """
        self.width = width
        self.height = height if height is not None else width  # Default to square if height is not given

    def display(self, particles=None):
        """
        Visualizes the 2D box along with optional particles.
        
        :param particles: A list of Particle2D objects to display inside the box.
        """
        fig, ax = plt.subplots()
        
        # If a custom boundary is defined, plot the polygon
        if self.boundary:
            boundary_points = np.array(self.boundary)
            boundary_polygon = plt.Polygon(boundary_points, closed=True, fill=None, edgecolor='r', linewidth=2)
            ax.add_patch(boundary_polygon)
            plt.xlim(min(boundary_points[:, 0]) - 1, max(boundary_points[:, 0]) + 1)
            plt.ylim(min(boundary_points[:, 1]) - 1, max(boundary_points[:, 1]) + 1)
        else:
            # Default to a rectangle or square box
            rectangle = plt.Rectangle((0, 0), self.width, self.height, fill=None, edgecolor='b', linewidth=2)
            ax.add_patch(rectangle)
            plt.xlim(-1, self.width + 1)
            plt.ylim(-1, self.height + 1)
        
        # Plot particles if any
        if particles:
            for particle in particles:
                circle = plt.Circle(particle.position, particle.radius, color='g', fill=True)
                ax.add_patch(circle)
        
        ax.set_aspect('equal', 'box')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('2D Box Container')
        plt.grid(True)
        plt.show()

    def is_within_bounds(self, particle):
        """
        Check if a given particle is within the bounds of the box.
        
        :param particle: A Particle2D object.
        :return: True if the particle is within the bounds, False otherwise.
        """
        x, y = particle.position
        # Simple rectangular bounds check
        if particle.radius <= x <= self.width - particle.radius and particle.radius <= y <= self.height - particle.radius:
            return True


    def __repr__(self):
        """
        Custom string representation for the Box2D object.
        """
        if self.boundary:
            return f"Box2D(boundary={self.boundary})"
        else:
            return f"Box2D(width={self.width}, height={self.height})"
