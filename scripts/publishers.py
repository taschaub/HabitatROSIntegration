import rospy
import math
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage, CameraInfo, LaserScan

def publish_rgb_image(observations, rgb_image_publisher):
    bridge = CvBridge()

    # Convert RGB data to a format suitable for saving as an image
    rgb_data = observations["rgb"]
    rgb_image_msg = bridge.cv2_to_imgmsg(rgb_data, encoding="bgr8")

    # Publish the RGB image
    rgb_image_publisher.publish(rgb_image_msg)

def publish_depth_image_and_camera_info(env, observations, depth_image_publisher, camera_info_publisher):
    # Your code for generating the depth image goes here
    # ...
    timestamp = rospy.Time.now()
    publish_depth_image(observations, depth_image_publisher, timestamp)
    publish_camera_info(env, camera_info_publisher, timestamp)
    
def publish_depth_image(observations, depth_image_publisher, timestamp):
    bridge = CvBridge()

    # Convert depth data to a format suitable for saving as an image
    #depth_data = (observations["depth"] * 255).astype(np.uint8)
    #depth_image_msg = bridge.cv2_to_imgmsg(depth_data, encoding="mono8")

    # Keep depth data as floating point values representing distance in meters
    depth_data = observations["depth"].astype(np.float32)
    depth_data*=10
    depth_image_msg = bridge.cv2_to_imgmsg(depth_data, encoding="32FC1")
    # Publish the depth image
    depth_image_msg.header.stamp = timestamp
    depth_image_publisher.publish(depth_image_msg)

def publish_camera_info(env, camera_info_publisher, timestamp):
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
