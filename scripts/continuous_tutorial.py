#!/usr/bin/env python3

import os
import random
import sys

import git
import magnum as mn
import numpy as np

import habitat_sim
from habitat_sim.utils import viz_utils as vut

if "google.colab" in sys.modules:
    os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"

repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
# %cd $dir_path
data_path = os.path.join(dir_path, "data")
output_path = os.path.join(
    dir_path, "examples/tutorials/managed_rigid_object_tutorial_output/"
)


def place_agent(sim):
    # place our agent in the scene
    agent_state = habitat_sim.AgentState()
    agent_state.position = [-0.15, -0.7, 1.0]
    agent_state.rotation = np.quaternion(-0.83147, 0, 0.55557, 0)
    agent = sim.initialize_agent(0, agent_state)
    return agent.scene_node.transformation_matrix()


def make_configuration():
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = os.path.join(
        data_path, "scene_datasets/habitat-test-scenes/apartment_1.glb"
    )
    assert os.path.exists(backend_cfg.scene_id)
    backend_cfg.enable_physics = True

    # sensor configurations
    # Note: all sensors must have the same resolution
    # setup 2 rgb sensors for 1st and 3rd person views
    camera_resolution = [544, 720]
    sensor_specs = []

    rgba_camera_1stperson_spec = habitat_sim.CameraSensorSpec()
    rgba_camera_1stperson_spec.uuid = "rgba_camera_1stperson"
    rgba_camera_1stperson_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgba_camera_1stperson_spec.resolution = camera_resolution
    rgba_camera_1stperson_spec.position = [0.0, 0.6, 0.0]
    rgba_camera_1stperson_spec.orientation = [0.0, 0.0, 0.0]
    rgba_camera_1stperson_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(rgba_camera_1stperson_spec)

    depth_camera_1stperson_spec = habitat_sim.CameraSensorSpec()
    depth_camera_1stperson_spec.uuid = "depth_camera_1stperson"
    depth_camera_1stperson_spec.sensor_type = habitat_sim.SensorType.DEPTH
    depth_camera_1stperson_spec.resolution = camera_resolution
    depth_camera_1stperson_spec.position = [0.0, 0.6, 0.0]
    depth_camera_1stperson_spec.orientation = [0.0, 0.0, 0.0]
    depth_camera_1stperson_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(depth_camera_1stperson_spec)

    rgba_camera_3rdperson_spec = habitat_sim.CameraSensorSpec()
    rgba_camera_3rdperson_spec.uuid = "rgba_camera_3rdperson"
    rgba_camera_3rdperson_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgba_camera_3rdperson_spec.resolution = camera_resolution
    rgba_camera_3rdperson_spec.position = [0.0, 1.0, 0.3]
    rgba_camera_3rdperson_spec.orientation = [-45, 0.0, 0.0]
    rgba_camera_3rdperson_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(rgba_camera_3rdperson_spec)

    # agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


def simulate(sim, dt=1.0, get_frames=True):
    # simulate dt seconds at 60Hz to the nearest fixed timestep
    print("Simulating " + str(dt) + " world seconds.")
    observations = []
    start_time = sim.get_world_time()
    while sim.get_world_time() < start_time + dt:
        sim.step_physics(1.0 / 60.0)
        if get_frames:
            observations.append(sim.get_sensor_observations())

    return observations

cfg = make_configuration()
try:  # Got to make initialization idiot proof
    sim.close()
except NameError:
    pass
sim = habitat_sim.Simulator(cfg)
agent_transform = place_agent(sim)

# get the primitive assets attributes manager
prim_templates_mgr = sim.get_asset_template_manager()

# get the physics object attributes manager
obj_templates_mgr = sim.get_object_template_manager()

# get the rigid object manager
rigid_obj_mgr = sim.get_rigid_object_manager()

sphere_template_id = obj_templates_mgr.load_configs(
    str(os.path.join(data_path, "test_assets/objects/sphere"))
)[0]

# add a sphere to the scene, returns the object
sphere_obj = rigid_obj_mgr.add_object_by_template_id(sphere_template_id)
# move sphere
sphere_obj.translation = [2.50, 0.0, 0.2]

# simulate
observations = simulate(sim, dt=1.5, get_frames=make_video)

if make_video:
    vut.make_video(
        observations,
        "rgba_camera_1stperson",
        "color",
        output_path + "sim_basics",
        open_vid=show_video,
    )