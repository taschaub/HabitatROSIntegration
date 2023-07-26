#!/usr/bin/env python3

import habitat
import habitat_sim
from queue import Queue
import rospy
from sensor_msgs.msg import Image as RosImage, CameraInfo, LaserScan
import tf2_ros
from threading import Thread
from habitat_sim_thread import habitat_sim_thread
from geometry_msgs.msg import Twist



from publish_test.msg import BasicAction


def main():
    topic = "cmd_vel"
    scene = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    agent_config = 0 # habitat.Simulator() .get_config(config_path="/home/aaron/catkin_ws/src/publish_test/src/config/website_config.yaml")

    action_queue = Queue()

    rospy.init_node("habitat_ros_bridge")

    depth_publisher = rospy.Publisher("depth_image", RosImage, queue_size=10)
    rgb_publisher = rospy.Publisher("rgb_image", RosImage, queue_size=10)
    camera_info_publisher = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
    # scan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)

    tf_broadcaster = tf2_ros.TransformBroadcaster()

              
    ht = Thread(target=habitat_sim_thread, args=(agent_config, scene, action_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster))
    ht.start()

    def callback(data):
        print("Action received:", data)
        action_queue.put(data)

    rospy.Subscriber(topic, Twist, callback)
    print("loaded")
    rospy.spin()

if __name__ == "__main__":
    main()
