import rospy
import subprocess
import signal
import os
from publish_test.msg import BasicAction
import time
import sys
from utils import temporary_subscribe
map_server_process = None
move_base_process = None
confirm_pub = None

def kill_subprocesses():
    global map_server_process, move_base_process
    if map_server_process:
        os.killpg(os.getpgid(map_server_process.pid), signal.SIGTERM)
    if move_base_process:
        os.killpg(os.getpgid(move_base_process.pid), signal.SIGTERM)

def signal_handler(sig, frame):
    kill_subprocesses()
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

def switch_scene_callback(msg):
    global map_server_process, move_base_process
    
    # Logic to switch scene in Habitat based on msg content

    if msg.ActionIdx == 0:
        # Restart map_server
        print("received_restart map")
        if map_server_process:
            os.killpg(os.getpgid(map_server_process.pid), signal.SIGTERM)
        map_yaml_path = "/home/aaron/catkin_ws/test_map.yaml"
        map_server_process = subprocess.Popen(['rosrun', 'map_server', 'map_server', map_yaml_path], preexec_fn=os.setsid)

    elif msg.ActionIdx == 1:
        # Restart move_base_turtle.launch
        if move_base_process:
            os.killpg(os.getpgid(move_base_process.pid), signal.SIGTERM)
        # move_base_process = subprocess.Popen(['roslaunch', 'my_robot_navigation', 'move_base_turtle.launch'], preexec_fn=os.setsid)
        move_base_process = subprocess.Popen(['roslaunch', 'arena_bringup', 'move_base_rosnav.launch'], preexec_fn=os.setsid)

        temporary_subscribe()
        confirm_cmd = BasicAction()
        confirm_cmd.Action = "STOP"
        confirm_cmd.ActionIdx = 0
        rospy.loginfo(confirm_cmd)
        confirm_pub.publish(confirm_cmd)
        # if move_base_process:
        #     try:
        #         rospy.loginfo("Killing move_base related nodes...")
        #         os.system("rosnode kill /rosnav_manager")  # Replace with your node names
        #         os.system("rosnode kill /move_base")
        #         os.system("rosnode kill /spacial_horizon_node")# Add more nodes if necessary
        #         time.sleep(4)  # Allow some time for nodes to shut down
                
        #         os.killpg(os.getpgid(move_base_process.pid), signal.SIGTERM)
        #         time.sleep(2)  # Allow some time for process to terminate
        #     except Exception as e:
        #         rospy.logerr(f"Error while killing nodes: {e}")

        # # Start the launch file again
        # move_base_process = subprocess.Popen(['roslaunch', 'arena_bringup', 'move_base_rosnav.launch'], preexec_fn=os.setsid)

def main():
    global map_server_process, move_base_process , confirm_pub

    rospy.init_node('map_launcher')
    confirm_pub = rospy.Publisher('confirm_move_base', BasicAction, queue_size=10)

    rospy.Subscriber("switch_scene", BasicAction, switch_scene_callback)
    # confirm_cmd = BasicAction()
    # confirm_cmd.Action = "STOP"
    # confirm_cmd.ActionIdx = 0
    # rospy.loginfo(confirm_cmd)
    # confirm_pub.publish(confirm_cmd)
    
    initial_map_path = "/home/aaron/catkin_ws/test_map.yaml"
    map_server_process = subprocess.Popen(['rosrun', 'map_server', 'map_server', initial_map_path], preexec_fn=os.setsid)
    # move_base_process = subprocess.Popen(['roslaunch', 'my_robot_navigation', 'move_base_turtle.launch'], preexec_fn=os.setsid)
    move_base_process = subprocess.Popen(['roslaunch', 'arena_bringup', 'move_base_rosnav.launch'], preexec_fn=os.setsid)

    rospy.spin()

if __name__ == "__main__":
    main()
