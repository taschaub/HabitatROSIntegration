"""
ROS node to manage map server and navigation processes, and switch between different scenes.
"""

import rospy
import subprocess
import signal
import os
import sys
import time
from publish_test.msg import BasicAction
from utils import temporary_subscribe

# Global variables
map_server_process = None
move_base_process = None
confirm_pub = None
scene_name = None

def kill_subprocesses():
    """Terminate map_server and move_base processes if they're running."""
    global map_server_process, move_base_process
    if map_server_process:
        os.killpg(os.getpgid(map_server_process.pid), signal.SIGTERM)
    if move_base_process:
        os.killpg(os.getpgid(move_base_process.pid), signal.SIGTERM)

def signal_handler(sig, frame):
    """Handler for SIGTERM and SIGINT signals to ensure subprocesses are terminated cleanly."""
    kill_subprocesses()
    sys.exit(0)

# Set up signal handlers
signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

def switch_scene_callback(msg):
    """
    Callback to switch between different scenes based on received message.
    
    Parameters:
    - msg: ROS message containing the desired action.
    """
    global map_server_process, move_base_process, scene_name

    if msg.ActionIdx == 0:
        # Logic to restart map_server with a new map
        scene_name = msg.Action
        if map_server_process:
            os.killpg(os.getpgid(map_server_process.pid), signal.SIGTERM)
        map_yaml_path = f"/home/aaron/catkin_ws/src/publish_test/evaluation/maps/{scene_name}.yaml"
        map_server_process = subprocess.Popen(['rosrun', 'map_server', 'map_server', map_yaml_path], preexec_fn=os.setsid)

    elif msg.ActionIdx == 1:
        # Logic to restart move_base launch file
        if move_base_process:
            os.killpg(os.getpgid(move_base_process.pid), signal.SIGTERM)
        
        move_base_process = subprocess.Popen(['roslaunch', 'arena_bringup', 'move_base_rosnav.launch'], preexec_fn=os.setsid)
        temporary_subscribe()

        confirm_cmd = BasicAction()
        confirm_cmd.Action = "STOP"
        confirm_cmd.ActionIdx = 0
        rospy.loginfo(confirm_cmd)
        confirm_pub.publish(confirm_cmd)

def main():
    """Main function to initialize the ROS node and manage map server and navigation processes."""
    global map_server_process, move_base_process, confirm_pub

    rospy.init_node('map_launcher')
    confirm_pub = rospy.Publisher('confirm_move_base', BasicAction, queue_size=10)
    rospy.Subscriber("switch_scene", BasicAction, switch_scene_callback)

    initial_map_path = "/home/aaron/catkin_ws/test_map.yaml"
    map_server_process = subprocess.Popen(['rosrun', 'map_server', 'map_server', initial_map_path], preexec_fn=os.setsid)
    move_base_process = subprocess.Popen(['roslaunch', 'arena_bringup', 'move_base_rosnav.launch'], preexec_fn=os.setsid)

    rospy.spin()

if __name__ == "__main__":
    main()
