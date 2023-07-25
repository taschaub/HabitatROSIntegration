#!/usr/bin/env python3

import habitat_sim
import os
import numpy as np
from publishers import publish_rgb_image, publish_depth_image_and_camera_info
from map import display_top_down_map
import cv2
import random
from habitat_sim.utils import viz_utils as vut
import git
from utils import make_configuration, init_locobot, print_screen, apply_cmd_vel


make_video = True
repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
# %cd $dir_path
data_path = os.path.join(dir_path, "data")
output_path = os.path.join(
    dir_path, "examples/tutorials/managed_rigid_object_tutorial_output/"
)


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

# # set the agent's position
# agent.state.position = [1.0, 0.0, 1.0]

# # set the agent's orientation
# agent.state.rotation = np.quaternion(1, 0, 0, 0)

locobot, vel_control = init_locobot(sim, obj_templates_mgr, rigid_obj_mgr)

 # reset observations and robot state
observations = []
locobot.translation = [3.75, -1.02, 4.4]
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
    display_top_down_map(sim)


    # # randomize angular velocity
    # last_velocity_set += time_step
    # if last_velocity_set >= 1.0:
    #     vel_control.angular_velocity = [0.0, (random.random() - 0.5) * 2.0, 0.0]
    #     last_velocity_set = 0

# video rendering with embedded 1st person views
if make_video:
    sensor_dims = (
        sim.get_agent(0).agent_config.sensor_specifications[0].resolution
    )
    overlay_dims = (int(sensor_dims[1] / 4), int(sensor_dims[0] / 4))
    overlay_settings = [
        {
            "obs": "rgba_camera",
            "type": "color",
            "dims": overlay_dims,
            "pos": (10, 10),
            "border": 2,
        },
        {
            "obs": "depth_camera",
            "type": "depth",
            "dims": overlay_dims,
            "pos": (10, 30 + overlay_dims[1]),
            "border": 2,
        },
    ]

    # vut.make_video(
    #     observations=observations,
    #     primary_obs="rgba_camera",
    #     primary_obs_type="color",
    #     video_file=output_path + video_prefix,
    #     fps=60,
    #     # open_vid=show_video,
    #     overlay_settings=overlay_settings,
    #     depth_clip=10.0,
    # )



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
