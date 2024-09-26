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

    def point_direction_method(self, vector = (11, 5)):
        desired_location = np.array(vector)
        for agent in self.agents:
            pointing_vector =  misc.normalize(desired_location - agent.position) * agent.desired_walking_speed
            agent.driving_force = np.array(agent.mass * (1 / self.time_constant) * (pointing_vector - agent.velocity))
            
            
            
            
            
    ###################################
    ### Repulsive force calculation ###
    ###################################
    
    def calculate_repulsive_forces(self, ellipse = True):
        eta = 0.3
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

                        
                        self.agents[i].repulsion_force += - (self.agents[i].mass * reduced_vision_factor * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / (separation - (d))) * separation_unit_vector  
                        # print(self.agents[i].repulsion_force)