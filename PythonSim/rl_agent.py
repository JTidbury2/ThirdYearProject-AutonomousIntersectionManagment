import logging 
import base64
import base64
from pathlib import Path

from gymnasium.wrappers import RecordVideo
from IPython import display as ipythondisplay
from pyvirtualdisplay import Display
from lib.settings import veh_dt
import os



import sys
from pathlib import Path

from rl_kinematics import Vehicle

# Get the parent directory of the current Jupyter notebook's directory
parent_dir = Path().resolve().parent

# Construct the path to the 'rl-agents' folder
rl_agents_dir = parent_dir / 'PettingZooSim/rl-agents'
print("rl_agnets,dric",rl_agents_dir)

# Add the 'rl-agents' directory to sys.path
sys.path.append(str(rl_agents_dir))

highway_env_dir = parent_dir / 'PettingZooSim/HighwayEnv'

sys.path.append(str(highway_env_dir))

from rl_agents.agents.common.factory import load_agent, load_environment


import highway_env

highway_env.register_highway_envs()


class AgentInference(object):
    def __init__(self,env_config, agent_config, model_path):
        os.chdir('../PettingZooSim/HighwayEnv')
        self.env=load_environment(env_config)
        self.agent = load_agent(agent_config, self.env)  # No environment needed for inference
        self.load_agent_model(model_path)
        self.agent.eval()
        os.chdir('../../PythonSim')

    def load_agent_model(self, model_path):
        if isinstance(model_path, str):
            model_path = Path(model_path)
        try:
            self.agent.load(filename=model_path)
            print(f"Loaded model from {model_path}")
        except FileNotFoundError:
            print("No pre-trained model found at the desired location.")
        except NotImplementedError:
            pass

    def get_action(self, state):
        # Assuming the agent has a method 'act' to get the action for a given observation
        action = self.agent.act(state,step_exploration_time=False)
        return action
    

    def translate_action(self, action):
        # Assuming the environment has a method to translate the action into a human-readable format
        actions = [self.agent.env.action_type.get_action(self.agent.env.action_type.getActionJames(a)) for a in action]
        return actions
    
    def get_agent_action(self, state):
        action = self.get_action(state)
        return self.translate_action([action])

class VehicleInterface(object):
    def __init__(self,position,speed, heading,):
        self.vehicle = Vehicle(position,speed, heading)
    def get_state_post_action(self):
        d= self.vehicle.to_dict()
        return {
            "presence": 1,
            "x": d["x"],
            "y": d["y"],
            "vx": d["vx"],
            "vy": d["vy"],
            "cosh": d["cos_h"],
            "sinh": d["sin_h"],
            "heading": d["heading"],
            "speed": d["speed"],
        }
    
    def vehicle_do_action(self, action):
        self.vehicle.act(action[0])
        self.vehicle.step(veh_dt)

    def get_state(self, action):
        self.vehicle_do_action(action)
        return self.get_state_post_action()
    
    def update_vehicle_values(self, position, speed, heading):
        self.vehicle.position = position
        self.vehicle.speed = speed
        self.vehicle.heading = heading
        
