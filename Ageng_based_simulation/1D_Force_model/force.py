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
    
    
    #Repulsive Force    
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
