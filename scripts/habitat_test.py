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

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

def display_top_down_map(env):
    agent_state = env.sim.get_agent_state()
    top_down_map = maps.get_topdown_map(env.sim.pathfinder, meters_per_pixel=0.05, height=0.5)
    top_down_map = maps.colorize_topdown_map(top_down_map)

    # Draw the agent's position and orientation on the map
    agent_pos = agent_state.position
    agent_rot = agent_state.rotation
    agent_angle = -2 * math.atan2(agent_rot.w, agent_rot.y)  # Get agent's yaw (rotation around the Z-axis)

    # Convert the agent position to integer values
    grid_resolution = 0.05
    bounds = env.sim.pathfinder.get_bounds()
    lower_bounds = bounds[0][[0, 2]]
    upper_bounds = bounds[1][[0, 2]]

    # Calculate the size of the map
    map_size = (upper_bounds - lower_bounds) / grid_resolution

    # Convert the agent position to grid coordinates
    agent_pos_grid = (agent_pos[[0, 2]] - lower_bounds) / grid_resolution
    agent_pos_int = np.round(agent_pos_grid).astype(np.int32)
    agent_pos_int_swapped = [agent_pos_int[1],agent_pos_int[0]]

    top_down_map_with_agent = maps.draw_agent(
        top_down_map,
        agent_pos_int_swapped,
        agent_angle,
        agent_radius_px=8,  # Adjust the agent_radius_px for the size of the agent marker
        # agent_color=(0, 0, 255)
    )
    cv2.imwrite("top_down_map.png", top_down_map_with_agent)
    #cv2.imshow("Top Down Map", top_down_map_with_agent)
    cv2.waitKey(1)

def publish_depth_and_rgb(observations):
    bridge = CvBridge()

    # Convert depth data to a format suitable for saving as an image
    depth_data = (observations["depth"] * 255).astype(np.uint8)
    depth_image_msg = bridge.cv2_to_imgmsg(depth_data, encoding="mono8")

    # Convert RGB data to a format suitable for saving as an image
    rgb_data = observations["rgb"]
    rgb_image_msg = bridge.cv2_to_imgmsg(rgb_data, encoding="bgr8")

    return depth_image_msg, rgb_image_msg

def habitat_thread(agent_config, scene, action_queue, depth_publisher, rgb_publisher):
    with read_write(agent_config):
        agent_config.habitat.simulator.scene = scene
    print(agent_config.habitat.simulator.scene)

    env = habitat.Env(agent_config)
    observations = env.reset()

    # Create a dictionary to map action indices to action names
    action_mapping = {
        0: 'stop',
        1: 'move_forward',
        2: 'turn left',
        3: 'turn right',
        4: 'print screen'
    }

    while not rospy.is_shutdown():

        display_top_down_map(env)

        depth_image_msg, rgb_image_msg = publish_depth_and_rgb(observations)
        depth_publisher.publish(depth_image_msg)
        rgb_publisher.publish(rgb_image_msg)

        # Print the current position and rotation
        agent_state = env.sim.get_agent_state()

        if not action_queue.empty():
            action_idx = action_queue.get()
            print("Action index received:", action_idx)

            # Get the action name from the action_mapping dictionary
            action_name = action_mapping.get(action_idx, None)
            if action_name is not None:
                if action_name == "print screen":
                    # Access the depth sensor data
                    depth_data = observations["depth"]

                    # Convert depth data to a format suitable for saving as an image
                    depth_image = (depth_data * 255).astype(np.uint8)

                    # Save the depth image to a file
                    cv2.imwrite("depth_image.png", depth_image)

                    # Display the depth image
                    #cv2.imshow("Depth Image", depth_image)
                    cv2.waitKey(1)

                else:
                    print("Executing action:", action_name)
                    observations = env.step(action_idx)
            else:
                print("Invalid action index:", action_idx)

        # Check if the episode is over and reset the environment
        if env.episode_over:
            print("Episode over. Resetting the environment.")
            observations = env.reset()

def main():
    topic = "chatter"
    scene = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    agent_config = habitat.get_config(config_path="/home/aaron/catkin_ws/src/publish_test/src/config/website_config.yaml")

    action_queue = Queue()

    rospy.init_node("habitat_ros_bridge")

    depth_publisher = rospy.Publisher("depth_image", Image, queue_size=10)
    rgb_publisher = rospy.Publisher("rgb_image", Image, queue_size=10)

    ht = Thread(target=habitat_thread, args=(agent_config, scene, action_queue, depth_publisher, rgb_publisher))
    ht.start()

    def callback(data):
        print("Action received:", data.ActionIdx)
        action_queue.put(data.ActionIdx)

    rospy.Subscriber(topic, BasicAction, callback)
    print("loaded")
    rospy.spin()

if __name__ == "__main__":
    main()
