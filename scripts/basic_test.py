#!/usr/bin/env python3
# 
import numpy as np
import habitat
from habitat.config.default import get_config
import random
from habitat.config.read_write import read_write

scene = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"


agent_config = habitat.get_config(config_path="/home/aaron/catkin_ws/src/publish_test/src/config/website_config.yaml")

with read_write(agent_config):
    agent_config.habitat.simulator.scene = scene
print(agent_config.habitat.simulator.scene)

# config = get_config(config_paths="config/habitat_all_sensors_test.yaml")
# print(config)

# env = habitat.Env(config=config)

env = habitat.Env(agent_config)
observations = env.reset()

action_mapping = {
    0: 'stop',
    1: 'move_forward',
    2: 'turn left',
    3: 'turn right'
}

max_steps = env._max_episode_steps
print(max_steps)
for i in range(len(env.episodes)):
    print("New Episode")
    observations = env.reset()
    # display_sample_rgbds(observations['rgb'], observations['semantic'], np.squeeze(observations['depth']))
    count_steps = 0
    while count_steps < max_steps:
        action = random.choice(list(action_mapping.keys()))
        print(action_mapping[action])
        observations = env.step(action)
        # display_sample_rgbds(observations['rgb'], observations['semantic'], np.squeeze(observations['depth']))
        count_steps += 1
        if env.episode_over:
            break
env.episodes = random.sample(env.episodes, 2)