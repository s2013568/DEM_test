from environment import Environment
from agent import Agent
from main import GCFModel
import measurement
import csv
import random


# Function to create agents evenly spaced across the field, avoiding overlap with the test agent
def create_agents(num_agents, canvas_width=26.0):
    agents = []

    # Calculate spacing between agents
    space_between_agents = (canvas_width) / (num_agents)

    current_x_position = 0.18
    for i in range(num_agents):
        # Create agents with the same properties except the test agent for the first position
        if i == 0:
            agents.append(Agent(position=[current_x_position, 1], velocity=[0.0, 0.0], a_min=0.18, b_min=0.1, tau=0.53, 
                                f=1.25, desired_walking_speed=1.34, test=True))
        else:
            agents.append(Agent(position=[current_x_position, 1], velocity=[0.0, 0.0], a_min=0.18, b_min=0.1, tau=0.53, 
                                f=1.25, desired_walking_speed=1.34, test=False))
        
        current_x_position += space_between_agents

    return agents



# Function to run a single simulation and log the test agent's velocity and density
def run_single_simulation(num_agents, steps=5000, dt=0.1, log_interval=1000):
    env = Environment(width=26, height=3, bottleneck_width=0, bottleneck_height=0, periodic=True)
    agents = create_agents(num_agents)

    
    gcf_model = GCFModel(environment=env, agents=agents, time_constant=0.5, eta=0.2)
    gcf_model.run_simulation(steps=steps, dt=dt, log_interval=log_interval)
    # gcf_model.animate(1, dt=0.01, interval=100, output_filename="crowd_simulation_test.gif", show_forces=True)
    
    print(agents[0].memory)
    # print(env.ins_density)
    # print(len(env.ins_density))
    # Calculate average velocity and density for the test agent
    v = measurement.find_average_velocity(agents[0])
    density = measurement.find_averaged_density(env.ins_density, agents[0], 0.01)
    
    return v, density

# Main function to run multiple simulations with varying number of agents
def run_multiple_simulations(agent_counts, steps=100001, dt=0.01, log_interval=1000):
    results = []
    
    for num_agents in agent_counts:
        print(f"Running simulation with {num_agents} agents...")
        v, density = run_single_simulation(num_agents, steps, dt, log_interval)
        results.append((num_agents, v, density))
        # print(num_agents, v, density)
    
    return results


# Function to write the simulation results to a CSV file
def write_results_to_csv(filename, results):
    # Write header and results to CSV
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Number of Agents", "Test Agent Velocity", "Test Agent Density"])
        writer.writerows(results)

# Define the number of agents to test (start with 10 agents and increase)
agent_counts = [
 60,
 55,
 50,
 50,
 48,
 46,
 44,
 42,
 40,
 38,
 36,
 34,
 32,
 30,
 30,
 29,
 28,
 27,
 26,
 25,
 24,
 23,
 22,
 21,
 20,
 19,
 18,
 17,
 16,
 15,
 14,
 13,
 12,
 11,
 10,
 9,
 8,
 7,
 6,
 5,
 4,
 3,
 2]
# agent_counts = [70]

# Run the simulations
results = run_multiple_simulations(agent_counts)
print(results)

# Save the results to a CSV file
csv_filename = "simulation_results_evenly_spaced.csv"
write_results_to_csv(csv_filename, results)

print(f"Results saved to {csv_filename}")
