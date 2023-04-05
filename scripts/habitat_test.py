#!/usr/bin/env python3

import rospy
import habitat
from publish_test.msg import BasicAction
from habitat.config.read_write import read_write
from threading import Thread
from queue import Queue

import math
import numpy as np
import cv2
from habitat.utils.visualizations import maps

def display_top_down_map(env):
    agent_state = env.sim.get_agent_state()
    top_down_map = maps.get_topdown_map(env.sim.pathfinder, meters_per_pixel=0.1, height=0.5)
    top_down_map = maps.colorize_topdown_map(top_down_map)

    # Draw the agent's position and orientation on the map
    agent_pos = agent_state.position
    agent_rot = agent_state.rotation
    agent_angle = -2 * math.atan2(agent_rot.y, agent_rot.w)  # Get agent's yaw (rotation around the Z-axis)

    # Convert the agent position to integer values
    grid_resolution = 0.1
    agent_pos_grid = (agent_pos - env.sim.pathfinder.get_bounds()[0]) / grid_resolution
    agent_pos_int = np.round(agent_pos_grid).astype(np.int32)

    top_down_map_with_agent = maps.draw_agent(
        top_down_map,
        agent_pos_int,
        agent_angle,
        agent_radius_px=8,  # Adjust the agent_radius_px for the size of the agent marker
        # agent_color=(0, 0, 255)
    )

    cv2.imshow("Top Down Map", top_down_map_with_agent)
    cv2.waitKey(1)

def habitat_thread(agent_config, scene, action_queue):
    with read_write(agent_config):
        agent_config.habitat.simulator.scene = scene
    print(agent_config.habitat.simulator.scene)

    env = habitat.Env(agent_config)
    observations = env.reset()

    while not rospy.is_shutdown():
        display_top_down_map(env)
        if not action_queue.empty():
            action = action_queue.get()
            print(action)
            observations = env.step(action)
            print(observations)



def main():
    topic = "chatter"
    scene = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    agent_config = habitat.get_config(config_path="/home/aaron/catkin_ws/src/publish_test/src/config/website_config.yaml")

    action_queue = Queue()

    ht = Thread(target=habitat_thread, args=(agent_config, scene, action_queue))
    ht.start()

    def callback(data):
        print("Action received:", data.ActionIdx)
        action_queue.put(data.ActionIdx)

    rospy.init_node("habitat_ros_bridge")
    rospy.Subscriber(topic, BasicAction, callback)
    print("loaded")
    rospy.spin()


if __name__ == "__main__":
    main()
   