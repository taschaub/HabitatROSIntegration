"""
ROS module to publish camera and depth images, and camera information from observations.
"""

import rospy
import math
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage, CameraInfo

def publish_rgb_image(observations, rgb_image_publisher):
    """
    Publishes RGB images from given observations to the specified publisher.
    
    Parameters:
    - observations: Dictionary containing camera data.
    - rgb_image_publisher: ROS publisher for RGB images.
    """
    bridge = CvBridge()
    timestamp = rospy.Time.now()

    # Convert RGB data to a format suitable for saving as an image
    rgb_data = observations["camera"]
    rgb_image_msg = bridge.cv2_to_imgmsg(rgb_data)
    rgb_image_msg.header.stamp = timestamp

    # Publish the RGB image
    rgb_image_publisher.publish(rgb_image_msg)

def publish_depth_image_and_camera_info(sim, observations, depth_image_publisher, camera_info_publisher):
    """
    Publishes depth images and camera information from given observations and simulator.
    
    Parameters:
    - sim: Simulation environment.
    - observations: Dictionary containing depth data.
    - depth_image_publisher: ROS publisher for depth images.
    - camera_info_publisher: ROS publisher for camera information.
    """
    timestamp = rospy.Time.now()
    publish_depth_image(observations, depth_image_publisher, timestamp)
    publish_camera_info(sim, camera_info_publisher, timestamp)
    
def publish_depth_image(observations, depth_image_publisher, timestamp):
    """
    Publishes depth images from given observations to the specified publisher.
    
    Parameters:
    - observations: Dictionary containing depth data.
    - depth_image_publisher: ROS publisher for depth images.
    - timestamp: ROS timestamp for the message header.
    """
    bridge = CvBridge()

    # Convert depth data to a format suitable for saving as an image with floating point values
    depth_data = observations["depth"].astype(np.float32)
    depth_image_msg = bridge.cv2_to_imgmsg(depth_data, encoding="32FC1")
    
    depth_image_msg.header.stamp = timestamp
    depth_image_publisher.publish(depth_image_msg)

def publish_camera_info(sim, camera_info_publisher, timestamp):
    """
    Publishes camera information from the simulation environment to the specified publisher.
    
    Parameters:
    - sim: Simulation environment.
    - camera_info_publisher: ROS publisher for camera information.
    - timestamp: ROS timestamp for the message header.
    """
    camera_config = sim._sensors["camera"]._spec
    camera_config.width = 480
    camera_config.height = 480

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
    camera_info.header.stamp = timestamp
    camera_info.header.frame_id = "camera_link"
    camera_info.width = camera_config.width
    camera_info.height = camera_config.height
    camera_info.distortion_model = "plumb_bob"
    camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]  # Assuming no distortion
    camera_info.K = camera_matrix
    camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Identity rotation matrix
    camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]  # Projection matrix

    camera_info_publisher.publish(camera_info)
