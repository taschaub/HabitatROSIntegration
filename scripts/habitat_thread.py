from habitat.config.read_write import read_write
import habitat
from gtego_map import GTEgoMap
from map_server import MapServer

import numpy as np

import rospy
from threading import Thread
from queue import Queue
import cv2


from map import display_top_down_map
from publishers import publish_rgb_image, publish_depth_image_and_camera_info
from transformations import publish_noisy_odom_transform, publish_map_odom_transform, publish_base_link_to_scan_transform

def habitat_thread(agent_config, scene, action_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster):
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
    
    # Create an instance of the GTEgoMap class
    depth_height = env._sim._sensor_suite.sensors["depth"].config.height
    depth_width = env._sim._sensor_suite.sensors["depth"].config.width
    ego_map = GTEgoMap(depth_H=depth_height, depth_W=depth_width)
    
    # Create an instance of the MapServer class
    map_server = MapServer()

    while not rospy.is_shutdown():

        display_top_down_map(env)
        
        depth_data = observations["depth"]
        ego_map_image = ego_map.get_observation(depth_data)# Generate the occupancy map here
        # pointcloud = ego_map.convert_to_pointcloud(depth_data)
        #visualize_pointcloud(pointcloud)
        #laserscan = convert_to_laserscan(pointcloud)
        
        # if laserscan is not None:
        #     scan_publisher.publish(laserscan)
        
        #convert ego map image to have correct number of channels
        explored_map = ego_map_image[:, :, 0] * 255 # maybe switched?
        obstacle_map = ego_map_image[:, :, 1] * 255

        rgb_image = np.zeros((*ego_map_image.shape[:2], 3), dtype=np.uint8)

        rgb_image[:, :, 1] = obstacle_map  # Green channel for obstacles
        rgb_image[:, :, 2] = explored_map
        
        # Update the map data and publish the map
        published_map = (ego_map_image[:, :, 0] - 0.5) * 2 * 127
        published_map = published_map.astype(np.int8)

        map_server.update_map_data(published_map)
        map_server.publish_map()
        
        #create a map server and publish the map in an ros topic
        # map_server = MapServer(obstacle_map)
        # map_server.publish_map()
        


        #cv2.imwrite('ego_map_image.png', rgb_image)
        # Save the occupancy map as an image file
        #cv2.imwrite('ego_map_image.png', ego_map_image * 255)

        # Publish the rgb and depth images
        publish_rgb_image(observations, rgb_publisher)
        
        publish_depth_image_and_camera_info(env, observations, depth_publisher, camera_info_publisher)
        # publish_transfonrms(env.sim.get_agent_state(), tf_broadcaster)
        publish_map_odom_transform(env.sim.get_agent_state(), tf_broadcaster)
        publish_noisy_odom_transform(env.sim.get_agent_state(), tf_broadcaster)
        publish_base_link_to_scan_transform(tf_broadcaster)
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
                    cv2.imwrite('ego_map_image.png', rgb_image)

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