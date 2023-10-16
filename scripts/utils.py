import rospy
import numpy as np
import cv2
import habitat_sim
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatusArray
from habitat_ros_integration.msg import BasicAction
import re

def convert_to_laserscan(xyz_camera, scan_height=0.5, height_tolerance=0.2, angle_min=-np.pi, angle_max=np.pi, range_min=0.0, range_max=10.0):
    """
    Convert a depth image to a simulated LaserScan message.
    Only points within a certain height range (scan_height +/- height_tolerance) are included in the scan.
    The scan covers the angles from angle_min to angle_max.
    """
    # Filter points based on the height range
    min_height, max_height = scan_height - height_tolerance, scan_height + height_tolerance
    points_in_range = np.logical_and(xyz_camera[:, 1] > min_height, xyz_camera[:, 1] < max_height)
    xyz_camera = xyz_camera[points_in_range]

    if xyz_camera.size == 0:
        rospy.logwarn("No points in point cloud within specified height range.")
        return None

    # Convert point cloud to polar coordinates
    ranges = np.sqrt(xyz_camera[:, 0]**2 + xyz_camera[:, 2]**2)
    angles = np.arctan2(xyz_camera[:, 0], xyz_camera[:, 2])

    # Create a LaserScan message
    num_measurements = angles.shape[0]
    angle_increment = (angle_max - angle_min) / num_measurements

    scan_msg = LaserScan()
    scan_msg.angle_min, scan_msg.angle_max = angle_min, angle_max
    scan_msg.angle_increment = angle_increment
    scan_msg.time_increment, scan_msg.range_min, scan_msg.range_max = 0.0, range_min, range_max  # Assuming instantaneous measurements
    scan_msg.ranges = ranges
    scan_msg.intensities = [1.0] * num_measurements  # Optional intensity value

    return scan_msg

def make_configuration(scene_path):
    """
    Make configuration for the simulator.
    Defines the backend and sensor configurations and returns a simulator configuration object.
    """
    # Backend configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = scene_path
    backend_cfg.enable_physics = True

    # Sensor configurations
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

    # Agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])

def get_rgb_and_depth_images(sim):
    """
    Retrieve RGB and Depth images from the simulator.
    """
    observations = sim.get_sensor_observations()
    return observations["camera"], observations["depth"]

def print_screen(rgb_image, depth_data):
    """
    Save and display RGB and Depth images.
    """
    # Convert depth data to image format and save
    depth_image = (depth_data * 8).astype(np.uint8)
    cv2.imwrite("depth_image.png", depth_image)
    cv2.imwrite('ego_map_image.png', rgb_image)

    # Optionally display the depth image
    cv2.waitKey(1)

# Additional functions init_robot(), discrete_vel_control(), etc. should be appropriately commented similarly.

def extract_scene_name(s):
    """
    Extract the scene name from the given string `s`.
    The string is expected to follow the pattern: 'data/scene_datasets/mp3d/{scene_name}/{scene_name}.glb'
    """
    match = re.search(r'data/scene_datasets/mp3d/([^/]+)/\1.glb', s)
    return match.group(1) if match else None
