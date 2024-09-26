import numpy as np
import misc

class Force:
    def __init__(self, agents, environment, desired_walking_speed = 1.3, time_constant = 1.0):
        self.agents = agents
        self.environment = environment
        self.desired_walking_speed = desired_walking_speed
        self.time_constant = time_constant



    ##########################################
    ### Driving force calculationg methods ###
    ##########################################

    def point_direction_method(self, line_points=((20, 0), (20, 10))):
        # Define the two points of the line
        point1 = np.array(line_points[0])
        point2 = np.array(line_points[1])
        
        # For each agent, calculate the nearest point on the line
        for agent in self.agents:
            # Get the agent's position
            agent_pos = agent.position
            
            # Vector from point1 to point2 (the direction of the line)
            line_vector = point2 - point1
            
            # Vector from point1 to agent position
            point_to_agent_vector = agent_pos - point1
            
            # Project the agent position vector onto the line vector to find the nearest point
            line_length_squared = np.dot(line_vector, line_vector)
            t = np.dot(point_to_agent_vector, line_vector) / line_length_squared
            
            # Clamp t between 0 and 1 to keep the nearest point on the line segment
            t = max(0, min(1, t))
            
            # Compute the nearest point on the line
            nearest_point_on_line = point1 + t * line_vector
            
            # Calculate the pointing vector (direction) to the nearest point
            pointing_vector = misc.normalize(nearest_point_on_line - agent_pos) * agent.desired_walking_speed
            
            # Compute the driving force
            agent.driving_force = agent.mass * (1 / self.time_constant) * (pointing_vector - agent.velocity)           
            
            
            
            
    ###################################
    ### Repulsive force calculation ###
    ###################################
    
    def calculate_repulsive_forces(self, ellipse = True, eta = 0.3):
        tau = 1.0
        if not ellipse:
            for i in range(len(self.agents)):
                self.agents[i].reset()
                for j in range(len(self.agents)):
                    if i != j:
                        separation = np.linalg.norm(self.agents[j].position - self.agents[i].position)
                        separation_unit_vector = (self.agents[j].position - self.agents[i].position) / separation
                        relative_velocity = (1 / 2) * (np.dot((self.agents[i].velocity - self.agents[j].velocity), separation_unit_vector) + abs(np.dot((self.agents[i].velocity - self.agents[j].velocity), separation_unit_vector)))
                        if np.linalg.norm(self.agents[i].velocity) != 0:
                            reduced_vision_factor = (np.dot(self.agents[i].velocity, separation_unit_vector) + abs(np.dot(self.agents[i].velocity, separation_unit_vector))) / (2 * np.linalg.norm(self.agents[i].velocity))
                        else:
                            reduced_vision_factor = 0

                        
                        self.agents[i].repulsion_force += - (self.agents[i].mass * reduced_vision_factor * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / (separation - (self.agents[i].radius + tau * self.agents[i].velocity) - (self.agents[j].radius + + tau * self.agents[i].velocity))) * separation_unit_vector  

        else:
            for i in range(len(self.agents)):
                self.agents[i].reset()
                for j in range(len(self.agents)):
                    if i != j:
                        separation = np.linalg.norm(self.agents[j].position - self.agents[i].position)
                        separation_unit_vector = (self.agents[j].position - self.agents[i].position) / separation
                        relative_velocity = (1 / 2) * (np.dot((self.agents[i].velocity - self.agents[j].velocity), separation_unit_vector) + abs(np.dot((self.agents[i].velocity - self.agents[j].velocity), separation_unit_vector)))
                        if np.linalg.norm(self.agents[i].velocity) != 0:
                            reduced_vision_factor = (np.dot(self.agents[i].velocity, separation_unit_vector) + abs(np.dot(self.agents[i].velocity, separation_unit_vector))) / (2 * np.linalg.norm(self.agents[i].velocity))
                        else:
                            reduced_vision_factor = 0
                        d = misc.closest_distance_between_ellipses(self.agents[i], self.agents[j])
                        # print(d)

                        
                        self.agents[i].repulsion_force += - (self.agents[i].mass * reduced_vision_factor * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / d) * separation_unit_vector  
                        # print(self.agents[i].repulsion_force)