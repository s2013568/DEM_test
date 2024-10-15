from environment import Environment
from agent import Agent
from main import Force_Model
import csv
import random


# Function to create agents evenly spaced across the field, avoiding overlap with the test agent
def create_agents(num_agents, canvas_width=200):
    agents = []

    # Calculate spacing between agents
    space_between_agents = (canvas_width) / (num_agents)
    print(space_between_agents)

    current_x_position = 0
    for i in range(num_agents):
        # Create agents with the same properties except the test agent for the first position
        if i == 0:
            agents.append(Agent(position=current_x_position + 1.0, velocity=0, a0=1, b0=0.1, tau=1, av = 0, test=True))
        else:
            agents.append(Agent(position=current_x_position, velocity=0, a0=1, b0=1, tau=1,av = 0, test=False))
        
        current_x_position += space_between_agents

    return agents



# Function to run a single simulation and log the test agent's velocity and density
def run_single_simulation(num_agents):
    env = Environment(width=200, height=3)
    agents = create_agents(num_agents)
    
    gcf_model = Force_Model(environment=env, agents=agents, parameters = (0.70, 0, 2, 0), x_min = 0, x_max = 26)
    #gcf_model.run_simulation(steps=1, dt=0.001, log_interval=1)
    # gcf_model.run_simulation(steps=500000, dt=0.001, log_interval=1000)
    gcf_model.run_simulation(steps=3000000, dt=0.01, log_interval=10000)
    # gcf_model.animate(1, dt=0.1, interval=1, output_filename="crowd_simulation_test.gif", show_forces=True, save_interval=1)
    
    # print(agents[0].memory)
    # print(agents[0].velocity)
    # print(agents[0].total_force)
    # print(agents[0].driving_force)
    # print(agents[0].repulsion_force)

# Main function to run multiple simulations with varying number of agents


# Define the number of agents to test (start with 10 agents and increase)
agent_counts = [60]

# Run the simulations
results = run_single_simulation(133)
