#!/usr/bin/env python3

from shutil import move
import rospy
import habitat
import habitat_sim
from queue import Queue
from threading import Thread
from sensor_msgs.msg import Image as RosImage, CameraInfo, LaserScan
from geometry_msgs.msg import Twist
from publish_test.msg import SetupHabitat, BasicAction
from geometry_msgs.msg import PoseStamped


import tf2_ros

from habitat_sim_thread import habitat_sim_thread

# Constants
TOPIC_CMD_VEL = "cmd_vel"
TOPIC_SETUP = "/setup_habitat"
TOPIC_CRASH = "crash_detect"
TOPIC_CRASH = "switch_scene"
TOPIC_DEPTH_IMAGE = "depth_image"
TOPIC_RGB_IMAGE = "rgb_image"
TOPIC_CAMERA_INFO = "camera_info"
# SCENE_PATH = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
# SCENE_PATH = "data/scene_datasets/mp3d/1LXtFkjw3qL/1LXtFkjw3qL.glb"
# SCENE_PATH = "data/scene_datasets/mp3d/TbHJrupSAjP/TbHJrupSAjP.glb"
# bad laser SCENE_PATH = "data/scene_datasets/mp3d/2n8kARJN3HM/2n8kARJN3HM.glb"
# SCENE_PATH = ""
# SCENE_PATH = "data/scene_datasets/mp3d/5LpN3gDmAk7/5LpN3gDmAk7.glb"# 2 etagen
# SCENE_PATH = "data/scene_datasets/mp3d/8WUmhLawc2A/8WUmhLawc2A.glb"
SCENE_PATH = "data/scene_datasets/mp3d/759xd9YjKW5/759xd9YjKW5.glb"



def start_habitat_thread(setup_queue, message_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster, goal_publisher, crash_publisher, scene_publisher, move_base_queue):
    ht = Thread(target=habitat_sim_thread, 
                args=(SCENE_PATH, setup_queue, message_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster, goal_publisher, crash_publisher, scene_publisher, move_base_queue))
    ht.start()

def main():
    rospy.init_node("habitat_ros_bridge")

    message_queue = Queue()
    setup_queue = Queue()
    move_base_queue = Queue()


    depth_publisher = rospy.Publisher(TOPIC_DEPTH_IMAGE, RosImage, queue_size=10)
    rgb_publisher = rospy.Publisher(TOPIC_RGB_IMAGE, RosImage, queue_size=10)
    camera_info_publisher = rospy.Publisher(TOPIC_CAMERA_INFO, CameraInfo, queue_size=10)
    goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
    crash_publisher = rospy.Publisher(TOPIC_CRASH, BasicAction, queue_size=10)
    scene_publisher = rospy.Publisher(TOPIC_CRASH, BasicAction, queue_size=10)


    tf_broadcaster = tf2_ros.TransformBroadcaster()

    start_habitat_thread(setup_queue, message_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster, goal_publisher, crash_publisher, scene_publisher, move_base_queue)

    def callback_cmd_vel(data):
        print("Action received:", data)
        message_queue.put(data)
        
    def callback_setup(data):
        print("New setup message received")
        setup_queue.put(data)
        print(data)
        
    def callback_move_base_confirm(data):
        print("New setup message received")
        move_base_queue.put(data)
        print(data)

    rospy.Subscriber(TOPIC_CMD_VEL, Twist, callback_cmd_vel)
    rospy.Subscriber(TOPIC_SETUP, SetupHabitat , callback_setup)
    rospy.Subscriber("confirm_move_base", BasicAction , callback_move_base_confirm)
    print("loaded")
    rospy.spin()

if __name__ == "__main__":
    main()
