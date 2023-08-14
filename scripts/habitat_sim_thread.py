from habitat.config.read_write import read_write
import habitat
import habitat_sim
from gtego_map import GTEgoMap
from map_server import MapServer
from geometry_msgs.msg import PoseStamped



import numpy as np

import rospy
from threading import Thread
from queue import Queue
import cv2

from map import display_top_down_map
from publishers import publish_rgb_image, publish_depth_image_and_camera_info
import transformations as tfs
from utils import make_configuration, init_robot, print_screen, discrete_vel_control

# Constants
DEPTH_HEIGHT = 480
DEPTH_WIDTH = 480
TIME_STEP = 1.0 / 6.0
TIME_STEP = 1.0 / 6.0
EPSILON = 1e-5

def habitat_sim_thread(scene, setup_queue, message_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster, goal_publisher):
    """Main function for the habitat simulator thread."""

    # Initialize simulator, agents and objects
    simulator, agent, obj_templates_mgr, rigid_obj_mgr = init_simulator_and_objects(DEPTH_WIDTH, DEPTH_HEIGHT)

    # Initialize rigid_robot
    rigid_robot, vel_control = init_robot(simulator, obj_templates_mgr, rigid_obj_mgr)

    # Initialize the ego map
    ego_map = GTEgoMap(depth_H=DEPTH_HEIGHT, depth_W=DEPTH_WIDTH)

    # Start simulation
    start_simulation(simulator, agent, rigid_robot, vel_control, ego_map, setup_queue, message_queue, depth_publisher, camera_info_publisher, tf_broadcaster, goal_publisher)



def init_simulator_and_objects(depth_width, depth_height):
    """Create and initialize the simulator, agent, and object managers."""

    cfg = make_configuration()
    simulator = habitat_sim.Simulator(cfg)
    agent = simulator.get_agent(0)

    # Get the primitive assets attributes manager
    prim_templates_mgr = simulator.get_asset_template_manager()

    # Get the physics object attributes manager
    obj_templates_mgr = simulator.get_object_template_manager()

    # Get the rigid object manager
    rigid_obj_mgr = simulator.get_rigid_object_manager()

    return simulator, agent, obj_templates_mgr, rigid_obj_mgr



def start_simulation(simulator, agent, rigid_robot, vel_control, ego_map, setup_queue, message_queue, depth_publisher, camera_info_publisher, tf_broadcaster, goal_publisher):
    """Start the simulation and main loop."""

    observations = simulator.get_sensor_observations()

    while not rospy.is_shutdown():

        display_top_down_map(simulator)

        current_time = rospy.Time.now()

        publish_transforms_and_images(simulator, observations, depth_publisher, camera_info_publisher, tf_broadcaster, current_time)

        if not message_queue.empty():
            process_cmd_vel_message(message_queue, rigid_robot, vel_control, simulator)

        if not setup_queue.empty():
            process_setup_message(setup_queue, rigid_robot, simulator, goal_publisher)  
            
        # Run any dynamics simulation
        simulator.step_physics(TIME_STEP)
        observations = simulator.get_sensor_observations()

        # Clear message queue
        message_queue.queue.clear()


def publish_transforms_and_images(simulator, observations, depth_publisher, camera_info_publisher, tf_broadcaster, current_time):
    """Publish transforms and images from the simulator."""

    publish_depth_image_and_camera_info(simulator, observations, depth_publisher, camera_info_publisher)
    tfs.publish_map_odom_transform(simulator, tf_broadcaster, current_time)
    tfs.publish_odom_baselink_transform(simulator.agents[0].state, tf_broadcaster, current_time)
    tfs.publish_base_link_to_scan_transform(tf_broadcaster, current_time)
    tfs.publish_origin_to_map_transform(tf_broadcaster, current_time)


def process_cmd_vel_message(message_queue, rigid_robot, vel_control, simulator):
    """Process command velocity message from the queue."""

    cmd_vel_data = message_queue.get()
    print("CMD_VEL received:", cmd_vel_data)

    linear_velocity_x = cmd_vel_data.linear.x  # forward motion
    linear_velocity_y = cmd_vel_data.linear.y  # sideways motion
    angular_velocity = cmd_vel_data.angular.z  # rotation

    # Convert to Habitat coordinate system
    habitat_linear_velocity_x = -linear_velocity_x
    habitat_linear_velocity_y = -linear_velocity_y
    habitat_angular_velocity = angular_velocity

    vel_control.linear_velocity = np.array([habitat_linear_velocity_y, 0, habitat_linear_velocity_x])
    vel_control.angular_velocity = np.array([0, habitat_angular_velocity, 0])

    apply_velocity_control(simulator, rigid_robot, vel_control)
    
def process_setup_message(setup_queue, rigid_robot, simulator, goal_publisher):
    #TODO: stop move base until new scene is setup
    
    setup_data = setup_queue.get()
    
    StartPosRos = setup_data.StartPoint
    
    rigid_robot.translation = tfs.position_ros_to_hab(StartPosRos.position)
    
    start_rotation_array = tfs.ros_quat_to_ros_array(StartPosRos.orientation)
    test_rot = tfs.ros_to_habitat_quaternion(start_rotation_array)
    
    rigid_robot.rotation = test_rot
    # Run any dynamics simulation
    simulator.step_physics(TIME_STEP)
    
    # TODO: set goal position
    # Publishing the goal
    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = "map"  # publishing the goal in the odom frame
    goal_msg.pose = setup_data.GoalPoint
    goal_publisher.publish(goal_msg)
    
    


def apply_velocity_control(simulator, rigid_robot, vel_control):
    """Apply velocity control and check for collisions."""

    previous_rigid_state = rigid_robot.rigid_state

    target_rigid_state = vel_control.integrate_transform(TIME_STEP, previous_rigid_state)

    end_pos = simulator.step_filter(previous_rigid_state.translation, target_rigid_state.translation)

    rigid_robot.translation = end_pos
    rigid_robot.rotation = target_rigid_state.rotation

    check_for_collision(previous_rigid_state, end_pos, target_rigid_state)


def check_for_collision(previous_rigid_state, end_pos, target_rigid_state):
    """Check if a collision has occurred."""

    dist_moved_before_filter = (target_rigid_state.translation - previous_rigid_state.translation).dot()
    dist_moved_after_filter = (end_pos - previous_rigid_state.translation).dot()

    collided = (dist_moved_after_filter + EPSILON) < dist_moved_before_filter

    if collided:
        print("ouuups you´ve crashed")
