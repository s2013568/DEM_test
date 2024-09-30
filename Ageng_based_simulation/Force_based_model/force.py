import numpy as np
import misc

class Force:
    def __init__(self, agents, environment, time_constant = 0.5):
        self.agents = agents
        self.environment = environment
        self.time_constant = time_constant



    ##########################################
    ### Driving force calculationg methods ###
    ##########################################

    def point_direction_method(self, line_points=((30, 0), (30, 20))):
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
            if line_length_squared == 0:
                pointing_vector = misc.normalize(point1 - agent_pos)
            else:
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
    
    def calculate_repulsive_forces(self, ellipse = True, eta = 0.2):
        tau = 1.0
        reps = 0.1
        rc = 2
        
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

                        
                        self.agents[i].repulsion_force += - (self.agents[i].mass * reduced_vision_factor * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / (separation - (self.agents[i].a + tau * self.agents[i].velocity) - (self.agents[j].a + tau * self.agents[i].velocity))) * separation_unit_vector  
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

                        if d < reps:
                            l_hat = misc.calculate_closest_distance(self.agents[i], self.agents[j])
                            Frep = (self.agents[i].mass * reduced_vision_factor * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / reps)
                            print(f'Frep{Frep}')
                            print(reduced_vision_factor)
                            self.agents[i].repulsion_force -= (((-2 * Frep) / (reps - l_hat)) * d + 3*Frep) * separation_unit_vector
                            print(self.agents[i].repulsion_force / Frep)
                            
                        elif d > rc - reps:
                            Fc = (self.agents[i].mass * reduced_vision_factor * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / (rc - reps))
                            self.agents[i].repulsion_force -= ((Fc) / (rc - reps)) * (d - rc + reps) * separation_unit_vector
                            
                        else:
                            self.agents[i].repulsion_force += - (self.agents[i].mass * reduced_vision_factor * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / d) * separation_unit_vector  
                        
    def calculate_wall_force(self, ellipse = True, eta_alt = 5):
        for agent in self.agents:
            agent.reset()
            wall_to_interact = 0
            D_mag = 999
            D = np.array([0, 0])
            P = np.array([0, 0])
            A = np.array([0, 0])
            B = np.array([0, 0])
            self.wallxy = (0, 0)
            
            for i, wall in enumerate(self.environment.walls):
                x_range, y_range = wall.get('x_range'), wall.get('y_range')
                pos = ((x_range[0], y_range[0]), (x_range[1], y_range[1]))
                # print(pos)
                d_mag, d_unit, p, a, b = misc.calculate_shortest_distance_between_point_and_line(agent, pos)
                # print(d_mag)
                if d_mag < D_mag:
                    D_mag = d_mag
                    D = d_unit
                    P = p
                    A = a
                    B = b
                    self.wallxy = (x_range, y_range)
                    if misc.det_numpy(A, B) < 0 :
                        A = b
                        B = a
            
        
        ### Transform the line vector into the frame of the ellipse
        ### Method adapted from 'Distance of Closest Approach of Ellipse and Line (Segment)' by Sean Curtis
            N = np.array([-D[1], D[0]])
            P_dash = np.array([-agent.a * N[0], - agent.b * N[1]])
            b_value = np.dot(N, P_dash) - misc.det_numpy(A, B)
            

            if D[1] == 0:
                if D[0] > 0:
                    Theta_R = -np.pi / 2  # Set to pi/2 if D[0] is positive
                else:
                    Theta_R = np.pi / 2  # Set to -pi/2 if D[0] is negative
            else:
                Theta_R = np.arctan(-(agent.b * D[0]) / (agent.a * D[1]))

            R = np.array([agent.a * np.cos(Theta_R,), agent.b * np.sin(Theta_R)])
            
            d_value = np.dot(N,  R) - misc.det_numpy(A, B)
            
            l = b_value - d_value
            
            if np.linalg.norm(agent.velocity) != 0:
                reduced_vision_factor = (np.dot(agent.velocity, D) + abs(np.dot(agent.velocity, N))) / (2 * np.linalg.norm(agent.velocity))
            else:
                reduced_vision_factor = 0
            
            alpha = np.arccos(np.dot(misc.normalize(p - agent.position), misc.normalize(agent.velocity)))
            
            q = ((np.cos(alpha) ** 2) / (agent.a ** 2)) + ((np.sin(alpha) ** 2) / (agent.b ** 2))

            
            r = np.sqrt(1 / q)
            
            biw = np.heaviside(1 - D_mag / (l + r), 1) * (1 - D_mag / (l + r))
            
            
            # print(D[1])
            # print(f'P:{P}')
            # print(f'D_mag:{D_mag}')
            # print(f'biw:{biw}')
            # print(f'l:{l}')
            # print(f'r:{r}')
            # print(f'b_value:{b_value}')
            # print(f'd_value:{d_value}')
            # print(self.wallxy)
            
            f = eta_alt * agent.desired_walking_speed * reduced_vision_factor * biw * misc.normalize(P - agent.position)
            # print(f'wall force: {f}')
            
            agent.repulsion_force -= f
            agent.add_force()