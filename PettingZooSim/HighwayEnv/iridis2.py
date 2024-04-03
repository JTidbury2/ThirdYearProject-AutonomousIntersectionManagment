from pathlib import Path

import highway_env

import sys

highway_env.register_highway_envs()

sys.path.insert(0, '/HighwayEnv/scripts/')


parent_dir = Path().resolve().parent

rl_agents_dir = parent_dir / 'rl-agents'

sys.path.append(str(rl_agents_dir))


from rl_agents.trainer.evaluation import Evaluation
from rl_agents.agents.common.factory import load_agent, load_environment

NUM_EPISODES = 1

env_config = '../rl-agents/scripts/configs/IntersectionEnv/env_3way_int.json'
agent_config = '../rl-agents/scripts/configs/IntersectionEnv/agents/DQNAgent/ego_attention_8h.json'

env = load_environment(env_config)
agent = load_agent(agent_config, env)
agent.load("iridisResult/run_20240331-000554_133973/run_20240331-000554_133973")


evaluation = Evaluation(env, agent, num_episodes=NUM_EPISODES, display_env=False, display_agent=False)
saved_dir=evaluation.train()
