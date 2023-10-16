#!/usr/bin/env python3

import rospy
import habitat
import habitat_sim
from queue import Queue
from threading import Thread
from sensor_msgs.msg import Image as RosImage, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from HabitatRosIntegration.msg import SetupHabitat, BasicAction
import tf2_ros
from habitat_sim_thread import habitat_sim_thread

# Constants
TOPIC_CMD_VEL = "cmd_vel"
TOPIC_SETUP = "/setup_habitat"
TOPIC_CRASH = "crash_detect"
TOPIC_SCENE = "switch_scene"
TOPIC_DEPTH_IMAGE = "depth_image"
TOPIC_RGB_IMAGE = "rgb_image"
TOPIC_CAMERA_INFO = "camera_info"
SCENE_PATH = "data/scene_datasets/mp3d/5LpN3gDmAk7/5LpN3gDmAk7.glb"  # Example scene path

def start_habitat_thread(setup_queue, message_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster, goal_publisher, crash_publisher, scene_publisher, move_base_queue):
    """Starts the Habitat simulation thread."""
    ht = Thread(target=habitat_sim_thread, 
                args=(SCENE_PATH, setup_queue, message_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster, goal_publisher, crash_publisher, scene_publisher, move_base_queue))
    ht.start()

def main():
    """Main function to initialize the ROS node and set up publishers, subscribers."""
    rospy.init_node("habitat_ros_bridge")

    # Initialize message queues
    message_queue = Queue()
    setup_queue = Queue()
    move_base_queue = Queue()

    # Initialize ROS publishers
    depth_publisher = rospy.Publisher(TOPIC_DEPTH_IMAGE, RosImage, queue_size=10)
    rgb_publisher = rospy.Publisher(TOPIC_RGB_IMAGE, RosImage, queue_size=10)
    camera_info_publisher = rospy.Publisher(TOPIC_CAMERA_INFO, CameraInfo, queue_size=10)
    goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
    crash_publisher = rospy.Publisher(TOPIC_CRASH, BasicAction, queue_size=10)
    scene_publisher = rospy.Publisher(TOPIC_SCENE, BasicAction, queue_size=10)

    # Initialize tf broadcaster for transformations
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Start the Habitat thread
    start_habitat_thread(setup_queue, message_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster, goal_publisher, crash_publisher, scene_publisher, move_base_queue)

    # Callback functions for ROS subscribers
    def callback_cmd_vel(data):
        """Callback for command velocity topic."""
        rospy.loginfo("Action received: %s", data)
        message_queue.put(data)
        
    def callback_setup(data):
        """Callback for Habitat setup topic."""
        rospy.loginfo("New setup message received: %s", data)
        setup_queue.put(data)
        
    def callback_move_base_confirm(data):
        """Callback for move base confirmation topic."""
        rospy.loginfo("New setup message received: %s", data)
        move_base_queue.put(data)

    # Initialize ROS subscribers
    rospy.Subscriber(TOPIC_CMD_VEL, Twist, callback_cmd_vel)
    rospy.Subscriber(TOPIC_SETUP, SetupHabitat, callback_setup)
    rospy.Subscriber("confirm_move_base", BasicAction, callback_move_base_confirm)

    rospy.loginfo("Habitat ROS bridge node loaded.")
    rospy.spin()

if __name__ == "__main__":
    main()
