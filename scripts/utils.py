import rospy
import numpy as np
from sensor_msgs.msg import LaserScan



def convert_to_laserscan(xyz_camera, scan_height=0.5, height_tolerance=0.2, angle_min=-np.pi, angle_max=np.pi, range_min=0.0, range_max=10.0):
    """
    Convert a depth image to a simulated LaserScan message.
    Only points within a certain height range (scan_height +/- height_tolerance) are included in the scan.
    The scan covers the angles from angle_min to angle_max.
    """
    # Filter the point cloud to include only points within the desired height range.
    min_height = scan_height - height_tolerance
    max_height = scan_height + height_tolerance
    points_in_range = np.logical_and(xyz_camera[:, 1] > min_height, xyz_camera[:, 1] < max_height)
    xyz_camera = xyz_camera[points_in_range]
    
     # Check if there are any points left after filtering.
    if xyz_camera.size == 0:
        rospy.logwarn("No points in point cloud within specified height range.")
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