from habitat.config.read_write import read_write
import habitat
import habitat_sim
from gtego_map import GTEgoMap
from map_server import MapServer

import numpy as np

import rospy
from threading import Thread
from queue import Queue
import cv2


from map import display_top_down_map
from publishers import publish_rgb_image, publish_depth_image_and_camera_info
from transformations import publish_odom_baselink_transform, publish_map_odom_transform, publish_base_link_to_scan_transform, publish_origin_to_map_transform
from utils import make_configuration, init_locobot, print_screen, apply_cmd_vel



def habitat_sim_thread(agent_config, scene, action_queue, depth_publisher, rgb_publisher, camera_info_publisher, tf_broadcaster):
    cfg = make_configuration()
    sim = habitat_sim.Simulator(cfg)
    
    time_step = 1.0 / 20.0
    
    action_mapping = {
        0: 'stop',
        1: 'move_forward',
        2: 'turn left',
        3: 'turn right',
        4: 'print screen'
    }

    agent = sim.get_agent(0)

    agent_transform = agent.scene_node.transformation_matrix()

    # get the primitive assets attributes manager
    prim_templates_mgr = sim.get_asset_template_manager()

    # get the physics object attributes manager
    obj_templates_mgr = sim.get_object_template_manager()

    # get the rigid object manager
    rigid_obj_mgr = sim.get_rigid_object_manager()
    
    locobot, vel_control = init_locobot(sim, obj_templates_mgr, rigid_obj_mgr)
    
    vel_control.linear_velocity = [0.0, 0.0, 0.0]

    
    observations = sim.get_sensor_observations()

    # Create an instance of the GTEgoMap class
    depth_height = 640
    depth_width = 480
    ego_map = GTEgoMap(depth_H=depth_height, depth_W=depth_width)
    
    # Create an instance of the MapServer class
    # map_server = MapServer()

    while not rospy.is_shutdown():

        display_top_down_map(sim)
        
        depth_data = observations["depth"]
        ego_map_image = ego_map.get_observation(depth_data)# Generate the occupancy map here
        # pointcloud = ego_map.convert_to_pointcloud(depth_data)
        #visualize_pointcloud(pointcloud)
        #laserscan = convert_to_laserscan(pointcloud)
        
        # if laserscan is not None:
        #     scan_publisher.publish(laserscan)
        
        #convert ego map image to have correct number of channels
        explored_map = ego_map_image[:, :, 0] * 255 # maybe switched?
        obstacle_map = ego_map_image[:, :, 1] * 255

        rgb_image = np.zeros((*ego_map_image.shape[:2], 3), dtype=np.uint8)

        rgb_image[:, :, 1] = obstacle_map  # Green channel for obstacles
        rgb_image[:, :, 2] = explored_map
        
        # Update the map data and publish the map
        published_map = (ego_map_image[:, :, 0] - 0.5) * 2 * 127
        published_map = published_map.astype(np.int8)

        # map_server.update_map_data(published_map)
        # map_server.publish_map()
        
        #create a map server and publish the map in an ros topic
        # map_server = MapServer(obstacle_map)
        # map_server.publish_map()
        
        #cv2.imwrite('ego_map_image.png', rgb_image)
        # Save the occupancy map as an image file
        #cv2.imwrite('ego_map_image.png', ego_map_image * 255)

        #get current time
        current_time = rospy.Time.now()
        
        # Publish the rgb and depth images
        # TODO error with encoding
        # publish_rgb_image(observations, rgb_publisher)
        
        publish_depth_image_and_camera_info(sim, observations, depth_publisher, camera_info_publisher)
        # publish_transfonrms(env.sim.get_agent_state(), tf_broadcaster)
        publish_map_odom_transform(sim, tf_broadcaster, current_time)
        publish_odom_baselink_transform(sim.agents[0].state, tf_broadcaster,current_time)
        publish_base_link_to_scan_transform(tf_broadcaster,current_time)
        publish_origin_to_map_transform(tf_broadcaster,current_time)
        # Print the current position and rotation
        agent_state = sim.agents[0].state
        
        if not action_queue.empty():
            action_idx = action_queue.get()
            print("Action index received:", action_idx)
            
            action_name = action_mapping.get(action_idx, None)
            if action_name == "move_forward":
                previous_rigid_state = locobot.rigid_state
                vel_control.linear_velocity = [0.0, 0.0, 1.0]

                # manually integrate the rigid state
                target_rigid_state = vel_control.integrate_transform(
                    time_step, previous_rigid_state
                )

                # snap rigid state to navmesh and set state to object/agent
                end_pos = sim.step_filter(
                    previous_rigid_state.translation, target_rigid_state.translation
                )
                locobot.translation = end_pos
                locobot.rotation = target_rigid_state.rotation

                # Check if a collision occured
                dist_moved_before_filter = (
                    target_rigid_state.translation - previous_rigid_state.translation
                ).dot()
                dist_moved_after_filter = (end_pos - previous_rigid_state.translation).dot()

                # NB: There are some cases where ||filter_end - end_pos|| > 0 when a
                # collision _didn't_ happen. One such case is going up stairs.  Instead,
                # we check to see if the the amount moved after the application of the filter
                # is _less_ than the amount moved before the application of the filter
                EPS = 1e-5
                collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter

                # run any dynamics simulation
                sim.step_physics(time_step)
                
            if action_name == "turn left":
                vel_control.angular_velocity = [0.0, 2.0, 0.0]
                
                previous_rigid_state = locobot.rigid_state

                # manually integrate the rigid state
                target_rigid_state = vel_control.integrate_transform(
                    time_step, previous_rigid_state
                )
                
                sim.step_physics(time_step)


                
            if action_name == "turn right":
                vel_control.angular_velocity = [0.0, -2.0, 0.0]
                
                previous_rigid_state = locobot.rigid_state

                # manually integrate the rigid state
                target_rigid_state = vel_control.integrate_transform(
                    time_step, previous_rigid_state
                )
                
                sim.step_physics(time_step)
            
            if action_name == "stop":
                last_velocity_set = 0
                vel_control.linear_velocity = [0.0, 0.0, 1.0]
                
            if action_name == "print screen":
                # Access the depth sensor data
                depth_data = observations["depth"]

                # Convert depth data to a format suitable for saving as an image
                depth_image = (depth_data * 255).astype(np.uint8)

                # Save the depth image to a file
                cv2.imwrite("depth_image.png", depth_image)
                # cv2.imwrite('ego_map_image.png', rgb_image)

                # Display the depth image
                #cv2.imshow("Depth Image", depth_image)
                cv2.waitKey(1)
                
        
        

        

    