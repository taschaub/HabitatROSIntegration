#!/usr/bin/env python3

import cv2
import numpy as np
import habitat
from habitat.config.default import get_config
from habitat.config.read_write import read_write


from habitat.tasks.nav.shortest_path_follower import ShortestPathFollower
from habitat.utils.visualizations import maps

scene = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
agent_config = habitat.get_config(config_path="/home/aaron/catkin_ws/src/publish_test/src/config/website_config.yaml")

with read_write(agent_config):
    agent_config.habitat.simulator.scene = scene
print(agent_config.habitat.simulator.scene)

env = habitat.Env(agent_config)

agent_state = env.sim.get_agent_state()
top_down_map = maps.get_topdown_map(env.sim.pathfinder, meters_per_pixel=0.1, height=0.5)

top_down_map = maps.colorize_topdown_map(top_down_map)

cv2.imshow("Top Down Map", top_down_map)
cv2.waitKey(0)

env.close()
