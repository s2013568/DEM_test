import numpy as np

class Force:
    def __init__(self, agents, environment, tau=0.5, mu=0.2, sigma=1, q=1):
        self.agents = agents
        self.environment = environment
        self.tau = tau
        self.mu = mu
        self.sigma = sigma
        self.q = q

    # Driving Force (now dimensional)
    def direction_1D_method(self):
        for agent in self.agents:
            # Driving force computation using dimensional variables
            agent.driving_force = (1 / self.tau) * (agent.v0 - agent.velocity)

    # Algebraically decaying repulsive force (now dimensional)
    def algebraically_decaying_repulsive(self, i, j, stage2=False):
        def ramp_function(x):
            if x >= 0:
                return 0
            else:
                return -x
        
        def apply_periodic_distance(agent_i_pos, agent_j_pos):
            """Calculate minimum separation between two agents with periodic boundaries."""
            applied = False
            dx = agent_j_pos - agent_i_pos

            # Apply periodic boundaries along the x-axis
            if abs(dx) > self.environment.width / 2:
                dx = dx - np.sign(dx) * self.environment.width
                applied = True

            return dx, applied

        if not stage2:
            # Dimensional velocity and separation calculations
            v0 = self.agents[i].v0

            separation, applied = apply_periodic_distance(self.agents[i].position, self.agents[j].position)
            relative_velocity = self.agents[i].velocity - self.agents[j].velocity

            # Dimensional distance calculation
            d = separation - self.agents[i].a - self.agents[j].a

            # Dimensional force computation
            self.agents[i].k1 = -(((self.mu + self.sigma * ramp_function(relative_velocity)) ** 2) / (d ** self.q)) + v0 - self.agents[i].velocity

        else:
            # Stage 2 calculations with buffered values (dimensional)
            v0 = self.agents[i].v0

            separation, applied = apply_periodic_distance(self.agents[i].buffer.get('position'), self.agents[j].position)
            relative_velocity = self.agents[i].buffer.get('velocity') - self.agents[j].velocity

            d = separation - self.agents[i].buffer.get('a') - self.agents[j].a

            # Dimensional k2 calculation
            k2 = -(((self.mu + self.sigma * ramp_function(relative_velocity)) ** 2) / (d ** self.q)) + v0 - self.agents[i].buffer.get('velocity')

            # Total force (dimensional)
            self.agents[i].total_force = (self.agents[i].k1 + k2) / 2
