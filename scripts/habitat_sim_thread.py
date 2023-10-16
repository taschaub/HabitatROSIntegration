from habitat.config.read_write import read_write
import habitat
import habitat_sim
from gtego_map import GTEgoMap
from map_server import MapServer
from geometry_msgs.msg import PoseStamped
from publish_test.msg import BasicAction



import numpy as np

import rospy
from threading import Thread
from queue import Queue
import cv2
from EpisodeManager import PoseTransformer


from map import display_top_down_map
from publishers import publish_rgb_image, publish_depth_image_and_camera_info
import transformations as tfs
from utils import make_configuration, init_robot, print_screen, discrete_vel_control, temporary_subscribe, extract_scene_name

# Constants
DEPTH_HEIGHT = 480
DEPTH_WIDTH = 480
TIME_STEP = 1.0 / 10.0
EPSILON = 1e-5

setup_state = None
setup_data = None
current_scene = None
short_scene = None
point_transformer = None

def habitat_sim_thread(scene, setup_queue, message_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster, goal_publisher, crash_publisher, scene_publisher, move_base_queue):
    """Main function for the habitat simulator thread."""

    # Initialize simulator, agents and objects
    simulator, agent, obj_templates_mgr, rigid_obj_mgr = init_simulator_and_objects(scene, DEPTH_WIDTH, DEPTH_HEIGHT)

    # Initialize rigid_robot
    rigid_robot, vel_control = init_robot(simulator, obj_templates_mgr, rigid_obj_mgr)

    # Initialize the ego map
    ego_map = GTEgoMap(depth_H=DEPTH_HEIGHT, depth_W=DEPTH_WIDTH)

    # Start simulation
    start_simulation(simulator, agent, rigid_robot, vel_control, ego_map, setup_queue, message_queue, depth_publisher,rgb_publisher, camera_info_publisher, tf_broadcaster, goal_publisher, crash_publisher, scene_publisher, move_base_queue)



def init_simulator_and_objects(scene, depth_width, depth_height):
    """Create and initialize the simulator, agent, and object managers."""

    cfg = make_configuration(scene)
    simulator = habitat_sim.Simulator(cfg)
    agent = simulator.get_agent(0)

    # Get the primitive assets attributes manager
    prim_templates_mgr = simulator.get_asset_template_manager()

    # Get the physics object attributes manager
    obj_templates_mgr = simulator.get_object_template_manager()

    # Get the rigid object manager
    rigid_obj_mgr = simulator.get_rigid_object_manager()

    return simulator, agent, obj_templates_mgr, rigid_obj_mgr



def start_simulation(simulator, agent, rigid_robot, vel_control, ego_map, setup_queue, message_queue, depth_publisher,rgb_publisher, camera_info_publisher, tf_broadcaster, goal_publisher, crash_publisher, scene_publisher, move_base_queue):
    """Start the simulation and main loop."""

    observations = simulator.get_sensor_observations()

    while not rospy.is_shutdown():

        display_top_down_map(simulator)

        current_time = rospy.Time.now()
        
        publish_transforms_and_images(simulator, observations, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster, current_time)
    
        if not message_queue.empty():
            process_cmd_vel_message(message_queue, rigid_robot, vel_control, simulator, crash_publisher)

        
        
        # if not setup_queue.empty():
        #     current_time.nsecs +=1000
        #     process_setup_message(setup_queue, rigid_robot, simulator, goal_publisher, tf_broadcaster, current_time)  
        simulator, agent , rigid_robot, vel_control = process_setup_message(setup_queue, rigid_robot, simulator, goal_publisher, tf_broadcaster, current_time, scene_publisher, agent, vel_control, move_base_queue)  


            
        # Run any dynamics simulation
        simulator.step_physics(TIME_STEP)
        observations = simulator.get_sensor_observations()

        # Clear message queue
        message_queue.queue.clear()


def publish_transforms_and_images(simulator, observations, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster, current_time):
    """Publish transforms and images from the simulator."""
    publish_rgb_image(observations, rgb_publisher)
    publish_depth_image_and_camera_info(simulator, observations, depth_publisher, camera_info_publisher)
    tfs.publish_map_odom_transform(simulator, tf_broadcaster, current_time)
    tfs.publish_odom_baselink_transform(simulator.agents[0].state, tf_broadcaster, current_time)
    tfs.publish_base_link_to_scan_transform(tf_broadcaster, current_time)
    tfs.publish_origin_to_map_transform(simulator, tf_broadcaster, current_time)


def process_cmd_vel_message(message_queue, rigid_robot, vel_control, simulator, crash_publisher):
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

    apply_velocity_control(simulator, rigid_robot, vel_control, crash_publisher)
    
def process_setup_message(setup_queue, rigid_robot, simulator, goal_publisher, tf_broadcaster, current_time, scene_publisher, agent, vel_control, move_base_queue):
    #in order to start an episode correctly we first set the starting point and in the next iteration we set the goal
    global setup_state, setup_data, current_scene, short_scene, point_transformer

    if setup_state is None and not setup_queue.empty():

        # Start new setup
        setup_data = setup_queue.get()
        setup_state = "update_position"
        if current_scene != setup_data.SceneName:
            current_scene = setup_data.SceneName
            short_scene = extract_scene_name(current_scene)
            setup_state = "switch_scene"
        else:
            point_transformer = PoseTransformer()
            setup_state = "simple_wait"

    if setup_state == "switch_scene":
        #set false so that the whole program doesnt stop
        simulator.close(False)
        # Initialize simulator, agents and objects
        simulator, agent, obj_templates_mgr, rigid_obj_mgr = init_simulator_and_objects(setup_data.SceneName, DEPTH_WIDTH, DEPTH_HEIGHT)

        # Initialize rigid_robot
        rigid_robot, vel_control = init_robot(simulator, obj_templates_mgr, rigid_obj_mgr) 
        setup_state = "wait"
       
    elif setup_state == "wait":
        point_transformer = PoseTransformer()
        
        setup_state = "tf_available"
        
    elif setup_state == "tf_available":        
        setup_state = "set_position"
        
    elif setup_state == "set_position":
        #add transfromer for start pos
        StartPosRos = point_transformer.transform_pose(setup_data.StartPoint)
        
        # StartPosRos = setup_data.StartPoint
        rigid_robot.translation = tfs.position_ros_to_hab(StartPosRos.position)
        start_rotation_array = tfs.ros_quat_to_ros_array(StartPosRos.orientation)
        test_rot = tfs.ros_to_habitat_quaternion(start_rotation_array)
        rigid_robot.rotation = test_rot
        simulator.step_physics(TIME_STEP)
        setup_state = "update_map"
   
    elif setup_state == "update_map":
        #publish map update -> switch scene
        #TODO restart move base??
        scene_cmd = BasicAction()
        scene_cmd.Action = short_scene
        scene_cmd.ActionIdx = 0
        rospy.loginfo(scene_cmd)
        scene_publisher.publish(scene_cmd)
        setup_state = "restart_move_base"  
            
    elif setup_state == "restart_move_base":
         #publish map update -> switch scene
        #TODO restart move base??
        scene_cmd = BasicAction()
        scene_cmd.Action = "short_scene"
        scene_cmd.ActionIdx = 1
        rospy.loginfo(scene_cmd)
        scene_publisher.publish(scene_cmd)
        setup_state = "prewait"
        
    elif setup_state == "prewait":
        setup_state = "move_base_check"
        
    elif setup_state == "move_base_check":
        if not move_base_queue.empty():
            move_base_queue.get()
            move_base_queue.queue.clear()
            setup_state = "update_position"
      
    elif setup_state == "simple_wait":
        setup_state = "simple_wait2"
        
    elif setup_state == "simple_wait2":
        setup_state = "update_position"

    elif setup_state == "update_position":
        #add transfromer for start po        
        StartPosRos = point_transformer.transform_pose(setup_data.StartPoint)
        
        # StartPosRos = setup_data.StartPoint
        rigid_robot.translation = tfs.position_ros_to_hab(StartPosRos.position)
        start_rotation_array = tfs.ros_quat_to_ros_array(StartPosRos.orientation)
        test_rot = tfs.ros_to_habitat_quaternion(start_rotation_array)
        rigid_robot.rotation = test_rot
        simulator.step_physics(TIME_STEP)
        setup_state = "publish_goal"
        point_transformer = None


    # elif setup_state == "publish_transform":
    #     tfs.publish_odom_baselink_transform(simulator.agents[0].state, tf_broadcaster, current_time)
    #     setup_state = "publish_goal"
    elif setup_state == "publish_goal":
        setup_state = "publish_goal2"


    elif setup_state == "publish_goal2":
        # Publishing the goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose = setup_data.GoalPoint
        goal_publisher.publish(goal_msg)
        
        # Reset state
        setup_state = None
        setup_data = None
        
    return simulator, agent , rigid_robot, vel_control
    
    


def apply_velocity_control(simulator, rigid_robot, vel_control, crash_publisher):
    """Apply velocity control and check for collisions."""

    previous_rigid_state = rigid_robot.rigid_state

    target_rigid_state = vel_control.integrate_transform(TIME_STEP, previous_rigid_state)

    end_pos = simulator.step_filter(previous_rigid_state.translation, target_rigid_state.translation)

    rigid_robot.translation = end_pos
    rigid_robot.rotation = target_rigid_state.rotation

    check_for_collision(previous_rigid_state, end_pos, target_rigid_state, crash_publisher)


def check_for_collision(previous_rigid_state, end_pos, target_rigid_state, crash_publisher):
    """Check if a collision has occurred."""

    dist_moved_before_filter = (target_rigid_state.translation - previous_rigid_state.translation).dot()
    dist_moved_after_filter = (end_pos - previous_rigid_state.translation).dot()

    collided = (dist_moved_after_filter + EPSILON) < dist_moved_before_filter

    if collided:
        print("ouuups you´ve crashed")
        crash_cmd = BasicAction()
        crash_cmd.Action = "STOP"
        crash_cmd.ActionIdx = 0
        rospy.loginfo(crash_cmd)
        crash_publisher.publish(crash_cmd)
        
    