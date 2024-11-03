from Generalized_testing_framework.environment import Environment
from agent import Agent
from main import GCFModel

env = Environment(width=20, height=10, bottleneck_width=2, bottleneck_height=2)
agents = [Agent(position=[1.0, 1.0], velocity=[0.0, 0.0], radius=0.3),
            Agent(position=[2.0, 1.0], velocity=[0.0, 0.0], radius=0.3)]

gcf_model = GCFModel(environment=env, agents=agents)
gcf_model.animate(steps=100, dt=0.1, interval=100, output_filename="crowd_simulation_social_force.gif", show_forces = True)