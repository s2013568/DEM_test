import numpy as np
def find_average_velocity(test_agent):
    t_in, t_out, x_in, x_out = test_agent.memory.get('t_in'),test_agent.memory.get('t_out'), test_agent.memory.get('x_in'), test_agent.memory.get('x_out')
    v = np.sqrt((x_out - x_in) ** 2) / ((t_out - t_in) * 0.01)
    return v



def find_inst_time_density(test_agent, x_min, x_max, all_agents):
    a_tot = 0
    l = x_max - x_min
    for agent in all_agents:
        a_tot += 2 * agent.a
    rho = a_tot / l
    return rho
    
def find_averaged_density(ins_density_list, test_agent, dt):
    t_in, t_out = test_agent.memory.get('t_in'),test_agent.memory.get('t_out')
    ins_density_array = np.array(ins_density_list)
    sum = np.sum(ins_density_array * dt)
    return sum / ((t_out - t_in) * 0.01)