import rospy
import subprocess
import signal
import os
from publish_test.msg import BasicAction

map_server_process = None
move_base_process = None

def switch_scene_callback(msg):
    global map_server_process, move_base_process
    
    # Logic to switch scene in Habitat based on msg content

    if msg.ActionIdx == 0:
        # Restart map_server
        if map_server_process:
            os.killpg(os.getpgid(map_server_process.pid), signal.SIGTERM)
        map_yaml_path = "/home/aaron/catkin_ws/test_map.yaml"
        map_server_process = subprocess.Popen(['rosrun', 'map_server', 'map_server', map_yaml_path], preexec_fn=os.setsid)

    elif msg.ActionIdx == 1:
        # Restart move_base_turtle.launch
        if move_base_process:
            os.killpg(os.getpgid(move_base_process.pid), signal.SIGTERM)
        move_base_process = subprocess.Popen(['roslaunch', 'my_robot_navigation', 'move_base_turtle.launch'], preexec_fn=os.setsid)

def main():
    global map_server_process, move_base_process

    rospy.init_node('map_launcher')
    rospy.Subscriber("switch_scene", BasicAction, switch_scene_callback)
    
    initial_map_path = "/home/aaron/catkin_ws/test_map.yaml"
    map_server_process = subprocess.Popen(['rosrun', 'map_server', 'map_server', initial_map_path], preexec_fn=os.setsid)
    move_base_process = subprocess.Popen(['roslaunch', 'my_robot_navigation', 'move_base_turtle.launch'], preexec_fn=os.setsid)

    rospy.spin()

if __name__ == "__main__":
    main()
