#!/usr/bin/env python3

import rospy
import habitat
from publish_test.msg import BasicAction
# from publish_test.scripts.gtego_map import GTEgoMap 
from gtego_map import GTEgoMap
# why does "from gtego_map import GTEgoMap" not work???
from habitat.config.read_write import read_write
from threading import Thread
from queue import Queue

import math
import numpy as np
import cv2
from habitat.utils.visualizations import maps

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import geometry_msgs.msg
import tf2_ros
import tf

from map_server import MapServer

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

def publish_rgb_image(observations, rgb_image_publisher):
    bridge = CvBridge()

    # Convert RGB data to a format suitable for saving as an image
    rgb_data = observations["rgb"]
    rgb_image_msg = bridge.cv2_to_imgmsg(rgb_data, encoding="bgr8")

    # Publish the RGB image
    rgb_image_publisher.publish(rgb_image_msg)

def publish_depth_image(observations, depth_image_publisher):
    bridge = CvBridge()

    # Convert depth data to a format suitable for saving as an image
    depth_data = (observations["depth"] * 255).astype(np.uint8)
    depth_image_msg = bridge.cv2_to_imgmsg(depth_data, encoding="mono8")

    # Publish the depth image
    depth_image_publisher.publish(depth_image_msg)

def publish_camera_info(env, camera_info_publisher):
    camera_config = env._sim._sensor_suite.sensors["rgb"].config

    # Calculate the vertical field of view (vfov) using the aspect ratio and hfov
    aspect_ratio = float(camera_config.width) / float(camera_config.height)
    vfov = 2 * math.atan(math.tan(math.radians(camera_config.hfov / 2)) / aspect_ratio)
    vfov = math.degrees(vfov)

    # Calculate the camera matrix (intrinsics) using the focal length
    fx = camera_config.width / (2 * math.tan(math.radians(camera_config.hfov / 2)))
    fy = camera_config.height / (2 * math.tan(math.radians(vfov / 2)))
    cx = camera_config.width / 2
    cy = camera_config.height / 2
    camera_matrix = [fx, 0, cx, 0, fy, cy, 0, 0, 1]

    camera_info = CameraInfo()
    camera_info.header.stamp = rospy.Time.now()
    camera_info.header.frame_id = "camera_link"
    camera_info.width = camera_config.width
    camera_info.height = camera_config.height
    camera_info.distortion_model = "plumb_bob"
    camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]  # Assuming no distortion
    camera_info.K = camera_matrix
    camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Identity rotation matrix
    camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]  # Projection matrix

    camera_info_publisher.publish(camera_info)


def publish_transforms(agent_state, tf_broadcaster):
    transform = geometry_msgs.msg.TransformStamped()

    # Set the frame IDs
    transform.header.frame_id = "world"
    transform.child_frame_id = "habitat_agent"

    # Set the translation
    transform.transform.translation.x = agent_state.position[0]
    transform.transform.translation.y = agent_state.position[1]
    transform.transform.translation.z = agent_state.position[2]

    # Set the rotation
    quaternion = (agent_state.rotation.x, agent_state.rotation.y, agent_state.rotation.z, agent_state.rotation.w)
    euler_angles = tf.transformations.euler_from_quaternion(quaternion)
    quaternion = tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    # Set the timestamp
    transform.header.stamp = rospy.Time.now()

    # Publish the transform
    tf_broadcaster.sendTransform(transform)

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
        ego_map_image = ego_map.get_observation(depth_data)  # Generate the occupancy map here
        
        #convert ego map image to have correct number of channels
        explored_map = ego_map_image[:, :, 0] * 255
        obstacle_map = ego_map_image[:, :, 1] * 255

        rgb_image = np.zeros((*ego_map_image.shape[:2], 3), dtype=np.uint8)

        rgb_image[:, :, 1] = obstacle_map  # Green channel for obstacles
        rgb_image[:, :, 2] = explored_map
        
        # Update the map data and publish the map
        published_map = (ego_map_image[:, :, 1] - 0.5) * 2 * 127
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
        publish_depth_image(observations, depth_publisher)
        publish_camera_info(env, camera_info_publisher)
        publish_transforms(env.sim.get_agent_state(), tf_broadcaster)

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

def main():
    topic = "chatter"
    scene = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    agent_config = habitat.get_config(config_path="/home/aaron/catkin_ws/src/publish_test/src/config/website_config.yaml")

    action_queue = Queue()

    rospy.init_node("habitat_ros_bridge")

    depth_publisher = rospy.Publisher("depth_image", Image, queue_size=10)
    rgb_publisher = rospy.Publisher("rgb_image", Image, queue_size=10)
    camera_info_publisher = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
              
    ht = Thread(target=habitat_thread, args=(agent_config, scene, action_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster))
    ht.start()

    def callback(data):
        print("Action received:", data.ActionIdx)
        action_queue.put(data.ActionIdx)

    rospy.Subscriber(topic, BasicAction, callback)
    print("loaded")
    rospy.spin()

if __name__ == "__main__":
    main()
