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
    
    def strat_1(self, point1 = (23.6, 2), point2 = (27, 2)):
        point1 = np.array(point1)
        point2 = np.array(point2)
        for agent in self.agents:
            agent_pos = agent.position
            if agent_pos[0] < 23.6:
                pointing_vector = misc.normalize(point1 - agent_pos) * agent.desired_walking_speed
            else:
                pointing_vector = misc.normalize(point2 - agent_pos) * agent.desired_walking_speed
            agent.driving_force = agent.mass * (1 / self.time_constant) * (pointing_vector - agent.velocity) 

    def strat_2(self, point1 = (23.6, 2)):

        def point_position_relative_to_line(x1, y1, x2, y2, x3, y3):
            # Compute the cross product
            cross_product = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)

            # Determine the position of the point relative to the line
            if cross_product > 0:
                return 1
            elif cross_product < 0:
                return -1
            else:
                return 0
        point1 = np.array(point1)
        for agent in self.agents:
            agent_pos = agent.position
            point2 = np.array((agent_pos[0] + 10., agent_pos[1]))
            if agent_pos[0] < 23.6:
                if agent_pos[1] >= self.environment.height / 2:
                    coefficient = point_position_relative_to_line(agent_pos[0], agent_pos[1], point2[0], point2[1], self.environment.bottleneck.get('x_min'), self.environment.bottleneck.get('y_max'))
                    #print(f'Top, coefficient {coefficient}')
                    if coefficient >= 0:
                        pointing_vector = misc.normalize(point2 - agent_pos) * agent.desired_walking_speed
                    else:
                        pointing_vector = misc.normalize(point1 - agent_pos) * agent.desired_walking_speed

                else:
                    coefficient = point_position_relative_to_line(agent_pos[0], agent_pos[1], point2[0], point2[1], self.environment.bottleneck.get('x_min'), self.environment.bottleneck.get('y_min'))
                    #print(f'Bot, coefficient {coefficient}')
                    if coefficient <= 0:
                        pointing_vector = misc.normalize(point2 - agent_pos) * agent.desired_walking_speed
                    else:
                        pointing_vector = misc.normalize(point1 - agent_pos) * agent.desired_walking_speed
            else:
                pointing_vector = misc.normalize(point2 - agent_pos) * agent.desired_walking_speed

            agent.driving_force = agent.mass * (1 / self.time_constant) * (pointing_vector - agent.velocity) 

    def strat_3(self):
        x1, y1 = self.environment.bottleneck.get('x_min') - 0.2, self.environment.bottleneck.get('y_min') + 0.2
        x2, y2 = self.environment.bottleneck.get('x_min') - 0.2, self.environment.bottleneck.get('y_max') - 0.2
        x3, y3 = self.environment.bottleneck.get('x_max') + 0.2, self.environment.bottleneck.get('y_min') + 0.2
        x4, y4 = self.environment.bottleneck.get('x_max') + 0.2, self.environment.bottleneck.get('y_max') - 0.2

        def closest_point_on_line(agent_position, line_start, line_end):
            # Unpack the coordinates
            px, py = agent_position
            x1, y1 = line_start
            x2, y2 = line_end
            
            # Vector AB
            line_vec = np.array([x2 - x1, y2 - y1])
            # Vector AP
            point_vec = np.array([px - x1, py - y1])
            
            # Project point vector onto line vector
            line_length_squared = np.dot(line_vec, line_vec)
            if line_length_squared == 0:  # The line segment is a point
                return np.array([x1, y1])  # Closest point is one of the endpoints

            projection = np.dot(point_vec, line_vec) / line_length_squared
            
            # Clamping projection to the line segment
            projection = max(0, min(1, projection))
            
            # Closest point on the line segment
            closest_point = np.array([x1, y1]) + projection * line_vec
            return closest_point

        def shortest_distance_vector(agent_position, line_start, line_end):
            closest_point = closest_point_on_line(agent_position, line_start, line_end)
            # Calculate the distance vector
            distance_vector = closest_point - np.array(agent_position)
            return distance_vector
        
        for agent in self.agents:
            # Get the agent's position
            agent_pos = agent.position
            if agent_pos[0] < x1:
                distance_vector = shortest_distance_vector(agent_pos, (x1, y1), (x2, y2))
            else:
                distance_vector = shortest_distance_vector(agent_pos, (x3, y3), (x4, y4))

            pointing_vector = misc.normalize(distance_vector) * agent.desired_walking_speed
            agent.driving_force = agent.mass * (1 / self.time_constant) * (pointing_vector - agent.velocity)

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
            
    def direction_1D_method(self):
        for agent in self.agents:
            current_velocity = np.linalg.norm(agent.velocity)
            
            # Compute the driving force
            agent.driving_force = agent.mass * (1 / self.time_constant) * (agent.desired_walking_speed - current_velocity)  * np.array([1, 0])
            
            
            
            
    ###################################
    ### Repulsive force calculation ###
    ###################################
    
    def calculate_repulsive_forces_1D(self, ellipse = True, eta = 0.2):
        tau = 1.0
        reps = 0.1
        rc = 2
        catastrophy = False
        
        def apply_periodic_distance(agent_i_pos, agent_j_pos):
                """Calculate minimum separation between two agents with periodic boundaries."""
                dx = agent_j_pos[0] - agent_i_pos[0]
                dy = agent_j_pos[1] - agent_i_pos[1]

                applied = False

                # Apply periodic boundaries along the x-axis
                if abs(dx) > self.environment.width / 2:
                    dx = dx - np.sign(dx) * self.environment.width

                    applied = True

                # Apply periodic boundaries along the y-axis
                if abs(dy) > self.environment.height / 2:
                    dy = dy - np.sign(dy) * self.environment.height
                    applied = True

                return np.array([dx, dy]), applied

        for i in range(len(self.agents)):
            self.agents[i].reset()
            for j in range(len(self.agents)):
                if i != j:
                    
                    separation_vector, applied= apply_periodic_distance(self.agents[i].position, self.agents[j].position)
                    separation = np.linalg.norm(separation_vector)
                    separation_unit_vector = separation_vector / separation
                    relative_velocity = (1 / 2) * (np.dot((self.agents[i].velocity - self.agents[j].velocity), separation_unit_vector) + abs(np.dot((self.agents[i].velocity - self.agents[j].velocity), separation_unit_vector)))
                    
                    d = separation - self.agents[i].a - self.agents[j].a
                    # if self.agents[i].test:
                    # print(f'relative_velocity: {relative_velocity}')
                    #     print(d)

                    if np.linalg.norm(self.agents[i].velocity) != 0:
                        reduced_vision_factor = (np.dot(self.agents[i].velocity, separation_unit_vector) + abs(np.dot(self.agents[i].velocity, separation_unit_vector))) / (2 * np.linalg.norm(self.agents[i].velocity))
                    else:
                        reduced_vision_factor = 0
                        
                    # if applied:
                    #     print(reduced_vision_factor)
                    #     print(self.agents[j].position[0] - self.agents[i].position[0])
                    # if d > 0:
                    #     print(relative_velocity)
                    
                    if d > rc:
                        self.agents[i].repulsion_force += 0
                    elif d < reps:
                        Frep = (self.agents[i].mass * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / reps)
                        # print(f'velocity{relative_velocity}')
                        # print(Frep)
                        # Linearly interpolate between 3Frep at d=0 and 1Frep at d=reps
                        repulsion_factor = 3 - (2 * (d / reps))
                        
                        # Calculate the repulsion force
                        if d >= 0:
                            self.agents[i].repulsion_force -= reduced_vision_factor * (repulsion_factor * Frep) * separation_unit_vector
                        else:
                            self.agents[i].repulsion_force -= reduced_vision_factor * (3 * Frep) * separation_unit_vector
                            # print(f'd < 0 {(3 * Frep) * separation_unit_vector}')
                            

                        # print((repulsion_factor * Frep) * separation_unit_vector)

                    elif d > rc - reps and d < rc:
                        Fc = (self.agents[i].mass * reduced_vision_factor * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / (rc - reps))
                        self.agents[i].repulsion_force -= ((-Fc / (reps)) * (d - rc + reps) + Fc)* separation_unit_vector

                    else:
                        self.agents[i].repulsion_force += - (self.agents[i].mass * reduced_vision_factor * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / d) * separation_unit_vector
                        # print(self.agents[i].repulsion_force)
                    
            if abs(np.linalg.norm(self.agents[i].repulsion_force)) > 10:
                print('catastrophy')
                return True

            
        return False
                        
                          
                        
                        
    def calculate_repulsive_forces(self, ellipse = True, eta = 0.2):
        tau = 1.0
        reps = 0.1
        rc = 2
        
        def apply_periodic_distance(agent_i_pos, agent_j_pos):
                """Calculate minimum separation between two agents with periodic boundaries."""
                
                dx = agent_j_pos[0] - agent_i_pos[0]
                dy = agent_j_pos[1] - agent_i_pos[1]

                applied = False

                # Apply periodic boundaries along the x-axis
                if abs(dx) > self.environment.width / 2:
                    dx = dx - np.sign(dx) * self.environment.width

                    applied = True

                # Apply periodic boundaries along the y-axis
                if abs(dy) > self.environment.height / 2:
                    dy = dy - np.sign(dy) * self.environment.height
                    applied = True

                return np.array([dx, dy]), applied

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
                        
                        separation_vector, applied= apply_periodic_distance(self.agents[i].position, self.agents[j].position)
                        separation = np.linalg.norm(separation_vector)
                        separation_unit_vector = separation_vector / separation
                        relative_velocity = (1 / 2) * (np.dot((self.agents[i].velocity - self.agents[j].velocity), separation_unit_vector) + abs(np.dot((self.agents[i].velocity - self.agents[j].velocity), separation_unit_vector)))
                        if np.linalg.norm(self.agents[i].velocity) != 0:
                            reduced_vision_factor = (np.dot(self.agents[i].velocity, separation_unit_vector) + abs(np.dot(self.agents[i].velocity, separation_unit_vector))) / (2 * np.linalg.norm(self.agents[i].velocity))
                        else:
                            reduced_vision_factor = 0
                        d = misc.closest_distance_between_ellipses(self.agents[i], self.agents[j], self.environment)
                        # if d > 0:
                        #     print(relative_velocity)
                        if d < reps:
                            l_hat = misc.calculate_closest_distance(self.agents[i], self.agents[j])
                            Frep = (self.agents[i].mass * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / reps)
                            # print(f'Frep{Frep}')
                            # print(f'l_hat{l_hat}')
                            self.agents[i].repulsion_force -= (((-2 * Frep) / (reps - l_hat)) * d + 3*Frep) * separation_unit_vector
                        elif d > rc:
                            self.agents[i].repulsion_force += 0

                        elif d > rc - reps and d < rc:
                            Fc = (self.agents[i].mass * reduced_vision_factor * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / (rc - reps))
                            self.agents[i].repulsion_force -= ((Fc) / (rc - reps)) * (d - rc + reps) * separation_unit_vector
                            
                        else:
                            self.agents[i].repulsion_force += - (self.agents[i].mass * reduced_vision_factor * ((eta * self.agents[i].desired_walking_speed + relative_velocity) ** 2) / d) * separation_unit_vector  
                        
                        
                        
    def calculate_wall_force(self, ellipse = True, eta_alt = 5):
        for agent in self.agents:
            agent.reset_wall()
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
            
            agent.wall_force -= f