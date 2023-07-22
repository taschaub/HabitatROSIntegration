#!/usr/bin/env python3

import habitat_sim
import os
import numpy as np
from publishers import publish_rgb_image, publish_depth_image_and_camera_info
from map import display_top_down_map
import cv2
import random


def make_configuration():
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    backend_cfg.enable_physics = True

    # sensor configurations
    sensor_specs = []

    rgba_camera_spec = habitat_sim.CameraSensorSpec()
    rgba_camera_spec.uuid = "rgba_camera"
    rgba_camera_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgba_camera_spec.resolution = [480, 640]
    rgba_camera_spec.position = [0.0, 0.6, 0.0]
    rgba_camera_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(rgba_camera_spec)

    depth_camera_spec = habitat_sim.CameraSensorSpec()
    depth_camera_spec.uuid = "depth_camera"
    depth_camera_spec.sensor_type = habitat_sim.SensorType.DEPTH
    depth_camera_spec.resolution = [480, 640]
    depth_camera_spec.position = [0.0, 0.6, 0.0]
    depth_camera_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(depth_camera_spec)

    # agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])

def apply_cmd_vel(agent, cmd_vel, dt):
    # cmd_vel should be a tuple or list like (linear_velocity, angular_velocity)
    linear_velocity, angular_velocity = cmd_vel

    # get the agent's current state
    state = agent.get_state()

    # integrate the linear velocity to get the change in position
    dp = linear_velocity * dt
    state.position += dp

    # integrate the angular velocity to get the change in orientation
    da = angular_velocity * dt
    state.rotation *= np.quaternion(np.cos(da / 2), 0, np.sin(da / 2),0)

    # set the agent's state
    agent.set_state(state, reset_sensors=False)
    
def get_rgb_and_depth_images(sim):
    observations = sim.get_sensor_observations()
    rgb_img = observations["rgba_camera"]
    depth_img = observations["depth_camera"]
    return rgb_img, depth_img

def print_screen(rgb_image, depth_data):
    # depth_data = sim.get_sensor_observations("depth")

    # Convert depth data to a format suitable for saving as an image
    depth_image = (depth_data * 8).astype(np.uint8)

    # Save the depth image to a file
    cv2.imwrite("depth_image.png", depth_image)
    cv2.imwrite('ego_map_image.png', rgb_image)

    # Display the depth image
    #cv2.imshow("Depth Image", depth_image)
    cv2.waitKey(1)
    
def place_agent(sim):
    # place our agent in the scene
    agent_state = habitat_sim.AgentState()
    agent_state.position = [-0.15, -0.7, 1.0]
    agent_state.rotation = np.quaternion(-0.83147, 0, 0.55557, 0)
    agent = sim.initialize_agent(0, agent_state)
    return agent

def init_locobot():
    locobot_template_id = obj_templates_mgr.load_configs(
        "/home/aaron/rosxhab/habitat-sim/data/objects/locobot_merged"
    )[0]
    # add robot object to the scene with the agent/camera SceneNode attached
    locobot = rigid_obj_mgr.add_object_by_template_id(
        locobot_template_id, sim.agents[0].scene_node
    )
    initial_rotation = locobot.rotation

    # set the agent's body to kinematic since we will be updating position manually
    locobot.motion_type = habitat_sim.physics.MotionType.KINEMATIC

    # create and configure a new VelocityControl structure
    # Note: this is NOT the object's VelocityControl, so it will not be consumed automatically in sim.step_physics
    vel_control = habitat_sim.physics.VelocityControl()
    vel_control.controlling_lin_vel = True
    vel_control.lin_vel_is_local = True
    vel_control.controlling_ang_vel = True
    vel_control.ang_vel_is_local = True
    vel_control.linear_velocity = [0.0, 0.0, -1.0]
    
    return locobot , vel_control



# create the simulator
cfg = make_configuration()
sim = habitat_sim.Simulator(cfg)

agent = sim.get_agent(0)

agent_transform = agent.scene_node.transformation_matrix()

# get the primitive assets attributes manager
prim_templates_mgr = sim.get_asset_template_manager()

# get the physics object attributes manager
obj_templates_mgr = sim.get_object_template_manager()

# get the rigid object manager
rigid_obj_mgr = sim.get_rigid_object_manager()

# Load navmesh and generate pathfinder
# pathfinder = habitat_sim.PathFinder()
# navmesh_settings = habitat_sim.nav.NavMeshSettings()
# navmesh_settings.set_defaults()
# habitat_sim.nav.NavMeshBuilder.build_nav_mesh(cfg.scene_id, navmesh_settings, 'skokloster-castle.navmesh')

# pathfinder.load_nav_mesh('skokloster-castle.navmesh')

# Sample a navigable point and set the agent's state
# nav_point = pathfinder.get_random_navigable_point()
# agent.state.position = np.array([nav_point[0], nav_point[1], nav_point[2]])

# Update the agent's state
# sim.set_agent_state(0, agent.state)

# navmesh_settings = habitat_sim.nav.NavMeshSettings()
# navmesh_settings.set_defaults()
# habitat_sim.nav.NavMeshBuilder.build_nav_mesh(sim_cfg.scene_id, navmesh_settings, 'path/to/your/scene.navmesh')

# set the agent's position
agent.state.position = [1.0, 0.0, 1.0]

# set the agent's orientation
agent.state.rotation = np.quaternion(1, 0, 0, 0)

locobot, vel_control = init_locobot()
###

# # create an ObjectAttributes for the agent
# agent_attr = habitat_sim.attributes.ObjectAttributes()
# agent_attr.mass = 1.0  # set the mass to 1 kg
# #agent_attr.up = [0, 1, 0]  # the up direction is along the y-axis
# agent_attr.scale = [0.1, 0.1, 0.1]  # scale the agent down to avoid collisions with the environment

# # load the attributes into the simulator's object template library
# agent_template_id = sim.get_object_template_manager().register_template(agent_attr)

# # add an object to the scene using the agent's attributes
# agent_object_id = sim.add_object(agent_template_id)

# # now control the agent_object_id instead of the agent
# sim.set_translation(agent.state.position, agent_object_id)
# sim.set_rotation(agent.state.rotation, agent_object_id)


###

# get the agent


 # reset observations and robot state
observations = []
locobot.translation = [1.75, -1.02, 0.4]
vel_control.angular_velocity = [0.0, 0.0, 0.0]

video_prefix = "robot_control_sliding"
# turn sliding off for the 2nd pass
if 1:
    sim.config.sim_cfg.allow_sliding = False
    video_prefix = "robot_control_no_sliding"

# manually control the object's kinematic state via velocity integration
start_time = sim.get_world_time()
last_velocity_set = 0
dt = 6.0
time_step = 1.0 / 60.0
while sim.get_world_time() < start_time + dt:
    previous_rigid_state = locobot.rigid_state

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

    # render observation
    observations.append(sim.get_sensor_observations())

    # randomize angular velocity
    last_velocity_set += time_step
    if last_velocity_set >= 1.0:
        vel_control.angular_velocity = [0.0, (random.random() - 0.5) * 2.0, 0.0]
        last_velocity_set = 0

# video rendering with embedded 1st person views
if make_video:
    sensor_dims = (
        sim.get_agent(0).agent_config.sensor_specifications[0].resolution
    )
    overlay_dims = (int(sensor_dims[1] / 4), int(sensor_dims[0] / 4))
    overlay_settings = [
        {
            "obs": "rgba_camera_1stperson",
            "type": "color",
            "dims": overlay_dims,
            "pos": (10, 10),
            "border": 2,
        },
        {
            "obs": "depth_camera_1stperson",
            "type": "depth",
            "dims": overlay_dims,
            "pos": (10, 30 + overlay_dims[1]),
            "border": 2,
        },
    ]

    vut.make_video(
        observations=observations,
        primary_obs="rgba_camera_3rdperson",
        primary_obs_type="color",
        video_file=output_path + video_prefix,
        fps=60,
        open_vid=show_video,
        overlay_settings=overlay_settings,
        depth_clip=10.0,
    )



cmd_vel = [1.2, 2.5]
while 1:
    for _ in range(10):
        apply_cmd_vel(agent, cmd_vel, dt=1/60)  # assuming the simulator runs at 60Hz
        sim.step_physics(1/60)

        # get the sensor observations for the current step
        observations = sim.get_sensor_observations()

        # get the RGB and depth images
        rgb_img = observations["rgba_camera"]
        depth_img = observations["depth_camera"]
        display_top_down_map(sim)
        print_screen(rgb_img,depth_img)
        # process the images or publish them to ROS...
        # ...
