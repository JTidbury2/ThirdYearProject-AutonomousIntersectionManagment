import base64
from pathlib import Path

from gymnasium.wrappers import RecordVideo
from IPython import display as ipythondisplay
from pyvirtualdisplay import Display
import highway_env

highway_env.register_highway_envs()
# Visualisation utils

import sys

sys.path.insert(0, '/HighwayEnv/scripts/')
def show_videos(path="videos"):
    html = []
    for mp4 in Path(path).glob("*.mp4"):
        video_b64 = base64.b64encode(mp4.read_bytes())
        html.append(
            """<video alt="{}" autoplay
                      loop controls style="height: 400px;">
                      <source src="data:video/mp4;base64,{}" type="video/mp4" />
                 </video>""".format(
                mp4, video_b64.decode("ascii")
            )
        )
    ipythondisplay.display(ipythondisplay.HTML(data="<br>".join(html)))


NUM_EPISODES = 1
import sys
from pathlib import Path

# Get the parent directory of the current Jupyter notebook's directory
parent_dir = Path().resolve().parent

# Construct the path to the 'rl-agents' folder
rl_agents_dir = parent_dir / 'rl-agents'

# Add the 'rl-agents' directory to sys.path
sys.path.append(str(rl_agents_dir))


from rl_agents.trainer.evaluation import Evaluation
from rl_agents.agents.common.factory import load_agent, load_environment

# Get the environment and agent configurations from the rl-agents repository


env_config = '../rl-agents/scripts/configs/IntersectionEnv/env_3way_int.json'
agent_config = '../rl-agents/scripts/configs/IntersectionEnv/agents/DQNAgent/ego_attention_8h.json'

env = load_environment(env_config)
agent = load_agent(agent_config, env)

evaluation = Evaluation(env, agent, num_episodes=NUM_EPISODES, display_env=False, display_agent=False)
print("env_config", env_config)
print(f"Ready to train {agent} on {env}")

saved_dir=evaluation.train()

env = load_environment(env_config)

print("Env confid", env.config)
env.config["offscreen_rendering"] = True
agent = load_agent(agent_config, env)
agent.load(saved_dir)
evaluation = Evaluation(env, agent, num_episodes=1)
saved_dir=evaluation.train()
show_videos(evaluation.run_directory)