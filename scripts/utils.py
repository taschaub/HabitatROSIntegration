import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import cv2
import habitat_sim


def convert_to_laserscan(xyz_camera, scan_height=0.5, height_tolerance=0.2, angle_min=-np.pi, angle_max=np.pi, range_min=0.0, range_max=10.0):
    """
    Convert a depth image to a simulated LaserScan message.
    Only points within a certain height range (scan_height +/- height_tolerance) are included in the scan.
    The scan covers the angles from angle_min to angle_max.
    """
    # Filter the point cloud to include only points within the desired height range.
    min_height = scan_height - height_tolerance
    max_height = scan_height + height_tolerance
    points_in_range = np.logical_and(
        xyz_camera[:, 1] > min_height, xyz_camera[:, 1] < max_height)
    xyz_camera = xyz_camera[points_in_range]

    # Check if there are any points left after filtering.
    if xyz_camera.size == 0:
        rospy.logwarn(
            "No points in point cloud within specified height range.")
        return None

    # Convert point cloud to polar coordinates.
    ranges = np.sqrt(xyz_camera[:, 0]**2 + xyz_camera[:, 2]**2)
    angles = np.arctan2(xyz_camera[:, 0], xyz_camera[:, 2])

    # Create a LaserScan message.
    num_measurements = angles.shape[0]
    angle_increment = (angle_max - angle_min) / num_measurements

    scan_msg = LaserScan()
    scan_msg.angle_min = angle_min
    scan_msg.angle_max = angle_max
    scan_msg.angle_increment = angle_increment
    scan_msg.time_increment = 0.0  # Assume all measurements are instantaneous
    scan_msg.range_min = range_min
    scan_msg.range_max = range_max
    scan_msg.ranges = ranges
    scan_msg.intensities = [1.0] * num_measurements  # Optional

    return scan_msg


def make_configuration(scene_path):
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    # backend_cfg.scene_id = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    backend_cfg.scene_id = scene_path#"data/scene_datasets/mp3d/1LXtFkjw3qL/1LXtFkjw3qL.glb"

    
    backend_cfg.enable_physics = True

    # sensor configurations
    sensor_specs = []

    rgba_camera_spec = habitat_sim.CameraSensorSpec()
    rgba_camera_spec.uuid = "camera"
    rgba_camera_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgba_camera_spec.resolution = [480, 480]
    rgba_camera_spec.position = [0.0, 0.6, 0.0]
    rgba_camera_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(rgba_camera_spec)

    depth_camera_spec = habitat_sim.CameraSensorSpec()
    depth_camera_spec.uuid = "depth"
    depth_camera_spec.sensor_type = habitat_sim.SensorType.DEPTH
    depth_camera_spec.resolution = [480, 480]
    depth_camera_spec.position = [0.0, 0.6, 0.0]
    depth_camera_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(depth_camera_spec)

    # agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


# def apply_cmd_vel(agent, cmd_vel, dt): 
#     #TODO: remove
#     # cmd_vel should be a tuple or list like (linear_velocity, angular_velocity)
#     linear_velocity, angular_velocity = cmd_vel

#     # get the agent's current state
#     state = agent.get_state()

#     # integrate the linear velocity to get the change in position
#     dp = linear_velocity * dt
#     state.position += dp

#     # integrate the angular velocity to get the change in orientation
#     da = angular_velocity * dt
#     state.rotation *= np.quaternion(np.cos(da / 2), 0, np.sin(da / 2), 0)

#     # set the agent's state
#     agent.set_state(state, reset_sensors=False)


def get_rgb_and_depth_images(sim):
    observations = sim.get_sensor_observations()
    rgb_img = observations["camera"]
    depth_img = observations["depth"]
    return rgb_img, depth_img


def print_screen(rgb_image, depth_data):
    # depth_data = sim.get_sensor_observations("depth")

    # Convert depth data to a format suitable for saving as an image
    depth_image = (depth_data * 8).astype(np.uint8)

    # Save the depth image to a file
    cv2.imwrite("depth_image.png", depth_image)
    cv2.imwrite('ego_map_image.png', rgb_image)

    # Display the depth image
    # cv2.imshow("Depth Image", depth_image)
    cv2.waitKey(1)


# def place_agent(sim):
#     # place our agent in the scene
#     agent_state = habitat_sim.AgentState()
#     agent_state.position = [-0.15, -0.7, 1.0]
#     agent_state.rotation = np.quaternion(-0.83147, 0, 0.55557, 0)
#     agent = sim.initialize_agent(0, agent_state)
#     return agent


def init_robot(sim, obj_templates_mgr, rigid_obj_mgr):
    locobot_template_id = obj_templates_mgr.load_configs(
        "/home/aaron/rosxhab/habitat-sim/data/objects/locobot_merged"
    )[0]
    # add robot object to the scene with the agent/camera SceneNode attached
    rigid_robot = rigid_obj_mgr.add_object_by_template_id(
        locobot_template_id, sim.agents[0].scene_node
    )
    initial_rotation = rigid_robot.rotation

    # set the agent's body to kinematic since we will be updating position manually
    rigid_robot.motion_type = habitat_sim.physics.MotionType.KINEMATIC

    # create and configure a new VelocityControl structure
    # Note: this is NOT the object's VelocityControl, so it will not be consumed automatically in sim.step_physics
    vel_control = habitat_sim.physics.VelocityControl()
    vel_control.controlling_lin_vel = True
    vel_control.lin_vel_is_local = True
    vel_control.controlling_ang_vel = True
    vel_control.ang_vel_is_local = True
    vel_control.linear_velocity = [0.0, 0.0, 0.0]

    return rigid_robot, vel_control


def discrete_vel_control(sim, action_name, vel_control, rigid_robot, time_step):
    if action_name == "move_forward":
        previous_rigid_state = rigid_robot.rigid_state
        vel_control.linear_velocity = [0.0, 0.0, -1.0]
        # manually integrate the rigid state
        target_rigid_state = vel_control.integrate_transform(
            time_step, previous_rigid_state
        )

        # snap rigid state to navmesh and set state to object/agent
        end_pos = sim.step_filter(
            previous_rigid_state.translation, target_rigid_state.translation
        )
        rigid_robot.translation = end_pos
        rigid_robot.rotation = target_rigid_state.rotation

        # Check if a collision occured
        dist_moved_before_filter = (
            target_rigid_state.translation - previous_rigid_state.translation
        ).dot()
        dist_moved_after_filter = (
            end_pos - previous_rigid_state.translation).dot()

        # NB: There are some cases where ||filter_end - end_pos|| > 0 when a
        # collision _didn't_ happen. One such case is going up stairs.  Instead,
        # we check to see if the the amount moved after the application of the filter
        # is _less_ than the amount moved before the application of the filter
        EPS = 1e-5
        collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter

        # run any dynamics simulation
        sim.step_physics(time_step)
        observations = sim.get_sensor_observations()

    if action_name == "turn left":
        vel_control.angular_velocity = [0.0, 2.0, 0.0]

        previous_rigid_state = rigid_robot.rigid_state

        # manually integrate the rigid state
        target_rigid_state = vel_control.integrate_transform(
            time_step, previous_rigid_state
        )

        sim.step_physics(time_step)
        observations = sim.get_sensor_observations()

    if action_name == "turn right":
        vel_control.angular_velocity = [0.0, -2.0, 0.0]

        previous_rigid_state = rigid_robot.rigid_state

        # manually integrate the rigid state
        target_rigid_state = vel_control.integrate_transform(
            time_step, previous_rigid_state
        )

        sim.step_physics(time_step)
        observations = sim.get_sensor_observations()

    if action_name == "stop":
        last_velocity_set = 0
        vel_control.linear_velocity = [0.0, 0.0, 1.0]

    if action_name == "print screen":
        # Access the depth sensor data
        depth_data = observations["depth"]

        max_val = depth_data.max()

        # Convert depth data to a format suitable for saving as an image
        depth_image = ((depth_data/max_val)*255).astype(np.uint8)

        # Save the depth image to a file
        cv2.imwrite("depth_image.png", depth_image)
        # cv2.imwrite('ego_map_image.png', rgb_image)

        # Display the depth image
        # cv2.imshow("Depth Image", depth_image)
        cv2.waitKey(1)
