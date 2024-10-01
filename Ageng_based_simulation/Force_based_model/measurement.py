import numpy as np
def find_average_velocity(test_agent):
    t_in, t_out, x_in, x_out = test_agent.memory.get('t_in'),test_agent.memory.get('t_out'), test_agent.memory.get('x_in'), test_agent.memory.get('x_out')
    v = np.sqrt(x_out - x_in) / ((t_out - t_in) * 0.01)
    return v



def find_inst_time_density(environment, test_agent, x_min, x_max, current_time, all_agents):
    if environment.is_in_test_region(test_agent, x_min, x_max, current_time):
        N = 0
        l = x_max - x_min
        for agent in all_agents:
            if agent.position[0] <= x_max and agent.position[0] >= x_min:
                N += 1
        rho = N / l
        return rho
    
def find_averaged_density(ins_density_list, test_agent, dt):
    t_in, t_out = test_agent.memory.get('t_in'),test_agent.memory.get('t_out')
    ins_density_array = np.array(ins_density_list)
    sum = np.sum(ins_density_array * dt)
    return sum / ((t_out - t_in) * 0.01)