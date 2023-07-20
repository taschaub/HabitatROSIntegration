#!/usr/bin/env python3

import habitat_sim
import os
import numpy as np

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

# create the simulator
cfg = make_configuration()
sim = habitat_sim.Simulator(cfg)

# get the agent
agent = sim.get_agent(0)

# set the agent's position
agent.state.position = [1.0, 0.0, 1.0]

# set the agent's orientation
agent.state.rotation = np.quaternion(1, 0, 0, 0)

print("TEST")

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
    state.rotation *= np.quaternion(np.cos(da / 2), 0, 0, np.sin(da / 2))

    # set the agent's state
    agent.set_state(state, reset_sensors=False)
    
def get_rgb_and_depth_images(sim):
    observations = sim.get_sensor_observations()
    rgb_img = observations["rgba_camera"]
    depth_img = observations["depth_camera"]
    return rgb_img, depth_img



cmd_vel = [1.2, 2.5]

for _ in range(10):
    apply_cmd_vel(agent, cmd_vel, dt=1/60)  # assuming the simulator runs at 60Hz
    sim.step_physics(1/60)

    # get the sensor observations for the current step
    observations = sim.get_sensor_observations()

    # get the RGB and depth images
    rgb_img = observations["rgba_camera"]
    depth_img = observations["depth_camera"]

    # process the images or publish them to ROS...
    # ...
