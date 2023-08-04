#!/usr/bin/env python3

import rospy
import habitat
import habitat_sim
from queue import Queue
from threading import Thread
from sensor_msgs.msg import Image as RosImage, CameraInfo, LaserScan
from geometry_msgs.msg import Twist
from publish_test.msg import SetupHabitat

import tf2_ros

from habitat_sim_thread import habitat_sim_thread

# Constants
TOPIC_CMD_VEL = "cmd_vel"
TOPIC_SETUP = "setup_habitat"
TOPIC_DEPTH_IMAGE = "depth_image"
TOPIC_RGB_IMAGE = "rgb_image"
TOPIC_CAMERA_INFO = "camera_info"
SCENE_PATH = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"

def start_habitat_thread(setup_queue, message_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster):
    ht = Thread(target=habitat_sim_thread, 
                args=(SCENE_PATH, setup_queue, message_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster))
    ht.start()

def main():
    rospy.init_node("habitat_ros_bridge")

    message_queue = Queue()
    setup_queue = Queue()


    depth_publisher = rospy.Publisher(TOPIC_DEPTH_IMAGE, RosImage, queue_size=10)
    rgb_publisher = rospy.Publisher(TOPIC_RGB_IMAGE, RosImage, queue_size=10)
    camera_info_publisher = rospy.Publisher(TOPIC_CAMERA_INFO, CameraInfo, queue_size=10)

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    start_habitat_thread(setup_queue, message_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster)

    def callback_cmd_vel(data):
        print("Action received:", data)
        message_queue.put(data)
        
    def callback_setup(data):
        print("New setup message received")
        print(data)

    rospy.Subscriber(TOPIC_CMD_VEL, Twist, callback_cmd_vel)
    rospy.Subscriber(TOPIC_SETUP, SetupHabitat , callback_setup)
    print("loaded")
    rospy.spin()

if __name__ == "__main__":
    main()
