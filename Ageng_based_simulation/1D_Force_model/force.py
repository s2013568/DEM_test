import numpy as np
class Force:
    def __init__(self, agents, environment, tau = 0.5, mu = 0.2, sigma = 1, q = 1):
        self.agents = agents
        self.environment = environment
        self.tau = tau
        self.mu = mu
        self.sigma = sigma 
        self.q = q
        
    #Driving Force
    def direction_1D_method(self):
        for agent in self.agents:
            agent.driving_force =  (1 / self.tau) * (agent.v0 - agent.velocity) 
            
  
    def algebraically_decaying_repulsive(self, i, j, stage2 = False):
        def ramp_function(x):
            if x >= 0:
                return 0
            else:
                return -x
        
        def apply_periodic_distance(agent_i_pos, agent_j_pos):
                """Calculate minimum separation between two agents with periodic boundaries."""
                applied = False
                dx = agent_j_pos - agent_i_pos
                # print(agent_j_pos)
                # print(agent_i_pos)
                # print(dx)
                # Apply periodic boundaries along the x-axis
                if abs(dx) > self.environment.width / 2:
                    dx = dx - np.sign(dx) * self.environment.width
                    applied = True
                

                return dx, applied

        if not stage2:   
            v0_dash = self.agents[i].v0_dash
            
            separation, applied= apply_periodic_distance(self.agents[i].x_dash, self.agents[j].x_dash)
            relative_velocity = self.agents[i].velocity_dash - self.agents[j].velocity_dash
            
            d_dash = separation - self.agents[i].a_dash - self.agents[j].a_dash
            # print(d_dash)
            
            
            self.agents[i].k1 = - (((self.mu + self.sigma * ramp_function(relative_velocity)) ** 2) / (d_dash ** self.q)) + v0_dash - self.agents[i].velocity_dash
        else:
            v0_dash = self.agents[i].v0_dash
            
            separation, applied= apply_periodic_distance(self.agents[i].buffer.get('x_dash'), self.agents[j].x_dash)
            relative_velocity = self.agents[i].buffer.get('velocity_dash') - self.agents[j].velocity_dash
            
            d_dash = separation - self.agents[i].buffer.get('a_dash') - self.agents[j].a_dash
            
            
            k2 = -(((self.mu + self.sigma * ramp_function(relative_velocity)) ** 2) / (d_dash ** self.q)) + v0_dash - self.agents[i].buffer.get('velocity_dash')
            

            self.agents[i].total_force = (self.agents[i].k1 + k2) / 2




    def new_repulsive_force_heun(self, i, j, stage2=False, epsilon=0.1):
        def smooth_ramp_function(x, epsilon):
            # Differentiable approximation of the ramp function r_epsilon
            return epsilon * np.log(1 + np.exp(-x / epsilon))
        def apply_periodic_distance(agent_i_pos, agent_j_pos):
            """Calculate minimum separation between two agents with periodic boundaries."""
            applied = False
            dx = agent_j_pos - agent_i_pos
            # print(agent_j_pos)
            # print(agent_i_pos)
            # print(dx)
            # Apply periodic boundaries along the x-axis
            if abs(dx) > self.environment.width / 2:
                dx = dx - np.sign(dx) * self.environment.width
                applied = True
            

            return dx, applied
        # Define the constant c as per the paper
        c = np.e - 1  # e is the base of the natural logarithm
        
        if not stage2:
            # Stage 1: Predictor step
            v0_dash = self.agents[i].v0_dash
            
            # Calculate effective distance and apply periodic boundaries if needed
            separation, applied = apply_periodic_distance(self.agents[i].x_dash, self.agents[j].x_dash)
            
            # Compute R_n for stage 1
            a_i = self.agents[i].a_dash
            a_j = self.agents[j].a_dash
            R_n = smooth_ramp_function(separation / (a_i + a_j) - 1, epsilon)
            
            # Compute the predicted force
            k1 = -v0_dash * np.log(c * R_n + 1) - self.agents[i].velocity_dash + self.agents[i].v0_dash
            
            # Store the predicted force k1 for use in the corrector step (stage 2)
            self.agents[i].k1 = k1
        
        else:
            # # Stage 2: Corrector step
            # v0_dash = self.agents[i].v0_dash
            
            # # Use buffered values for the corrector step
            # separation, applied = apply_periodic_distance(self.agents[i].buffer.get('x_dash'), self.agents[j].x_dash)
            
            # # Compute R_n for the corrector step
            # a_i_buffer = self.agents[i].buffer.get('a_dash')
            # a_j = self.agents[j].a_dash
            # R_n = smooth_ramp_function(separation / (a_i_buffer + a_j) - 1, epsilon)
            
            # # Compute the corrected force
            # k2 = -v0_dash * np.log(c * R_n + 1) - self.agents[i].velocity_dash + self.agents[i].v0_dash
            
            # Final force using Heun's scheme (average of k1 and k2)
            # self.agents[i].total_force = (self.agents[i].k1 + k2) / 2
            self.agents[i].total_force = self.agents[i].k1