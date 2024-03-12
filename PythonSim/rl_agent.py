import logging 
import base64
import base64
from pathlib import Path

from gymnasium.wrappers import RecordVideo
from IPython import display as ipythondisplay
from pyvirtualdisplay import Display
from lib.settings import veh_dt



import sys
from pathlib import Path

# Get the parent directory of the current Jupyter notebook's directory
parent_dir = Path().resolve().parent

# Construct the path to the 'rl-agents' folder
rl_agents_dir = parent_dir / 'rl-agents'

# Add the 'rl-agents' directory to sys.path
sys.path.append(str(rl_agents_dir))

highway_env_dir = parent_dir / 'HighwayEnv'

sys.path.append(str(highway_env_dir))

from rl_agents.agents.common.factory import load_agent, load_environment

import highway_env



class AgentInference(object):
    def __init__(self,env_config, agent_config, model_path):
        env=load_environment(env_config)
        self.agent = load_agent(agent_config, env)  # No environment needed for inference
        self.load_agent_model(model_path)
        self.agent.eval()

    def load_agent_model(self, model_path):
        if isinstance(model_path, str):
            model_path = Path(model_path)
        try:
            self.agent.load(filename=model_path)
            logging.info(f"Loaded model from {model_path}")
        except FileNotFoundError:
            logging.warning("No pre-trained model found at the desired location.")
        except NotImplementedError:
            pass

    def get_action(self, state):
        # Assuming the agent has a method 'act' to get the action for a given observation
        action = self.agent.act(state,step_exploration_time=False)
        return action
    

    def translate_action(self, action):
        # Assuming the environment has a method to translate the action into a human-readable format
        actions = [self.agent.env.action_type.get_action(self.agent.env.getActionJames(a)) for a in action]
        return actions
    
    def get_agent_action(self, state):
        action = self.get_action(state)
        return self.translate_action(action)

class VehicleInterface(object):
    def __init__(self,speed, heading,):
        self.vehicle = None 
    def get_state_post_action(self):
        d= self.vehicle.to_dict()
        return d["x"], d["y"] , self.vehicle.speed, d["vx"], d["xy"], d["cos_h"], d["sin_h"]
    
    def vehicle_do_action(self, action):
        self.vehicle.act(action)
        self.vehicle.step(veh_dt)

    def get_state(self, action):
        self.vehicle_do_action(action)
        return self.get_state_post_action()
