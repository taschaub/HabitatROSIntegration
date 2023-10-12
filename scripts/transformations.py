import rospy
import numpy as np
import geometry_msgs.msg
import tf
from magnum import Quaternion as qtn

def habitat_to_ros_quaternion(agent_rotation_habitat):
    """
    Converts a quaternion from Habitat's format to ROS format.
    Habitat's format is (w, x, y, z) whereas ROS format is (x, y, z, w).
    
    Args:
    - agent_rotation_habitat: Habitat quaternion

    Returns:
    - agent_rotation_ros: ROS format quaternion
    """
    transform_quat_ros = tf.transformations.quaternion_from_euler(0, 0, np.pi / 2)
    agent_rotation_ros = tf.transformations.quaternion_multiply(
        [agent_rotation_habitat.x, -agent_rotation_habitat.z, agent_rotation_habitat.y, agent_rotation_habitat.w], transform_quat_ros
    )
    return agent_rotation_ros

def ros_quat_to_ros_array(ros_quat):
    """Convert ROS quaternion object to numpy array."""
    return np.array([ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w])

def ros_to_habitat_quaternion(agent_rotation_ros):
    """
    Converts a quaternion from ROS's format to Habitat's format.

    Args:
    - agent_rotation_ros: ROS format quaternion

    Returns:
    - habitat_qtn: Habitat format quaternion
    """
    transform_quat_ros_inv = tf.transformations.quaternion_from_euler(0, 0, -np.pi / 2)
    agent_quat_habitat = tf.transformations.quaternion_multiply(agent_rotation_ros, transform_quat_ros_inv)
    habitat_qtn = qtn([agent_quat_habitat[0], agent_quat_habitat[2], -agent_quat_habitat[1]], agent_quat_habitat[3])
    return habitat_qtn

def position_hab_to_ros(position_hab):
    """Convert position from Habitat format to ROS format."""
    position_ros = geometry_msgs.msg.Vector3()
    position_ros.x = position_hab[0]
    position_ros.y = -position_hab[2]
    position_ros.z = 0
    return position_ros

def position_ros_to_hab(position_ros):
    """Convert position from ROS format to Habitat format."""
    return np.array([position_ros.x, position_ros.z, -position_ros.y])

def publish_odom_baselink_transform(agent_state, tf_broadcaster, current_time):
    """
    Publishes the transformation from the 'odom' frame to the 'base_footprint' frame.
    """
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_footprint"
    
    agent_pos = agent_state.position
    agent_rot = agent_state.rotation

    transform.transform.translation = position_hab_to_ros(agent_pos)
    quaternion = habitat_to_ros_quaternion(agent_rot)
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    transform.header.stamp = current_time
    tf_broadcaster.sendTransform(transform)

def publish_map_odom_transform(sim, tf_broadcaster, current_time):
    """
    Publishes the transformation from the 'map' frame to the 'odom' frame.
    """
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.frame_id = "map"
    transform.child_frame_id = "odom"
    bounds = sim.pathfinder.get_bounds()
    pos =[-bounds[0][0], bounds[1][2], 0]

    transform.transform.translation.x = pos[0]
    transform.transform.translation.y = pos[1]
    transform.transform.translation.z = pos[2]
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    transform.header.stamp = current_time
    tf_broadcaster.sendTransform(transform)

def publish_base_link_to_scan_transform(tf_broadcaster, current_time):
    """
    Publishes the transformation from the 'base_footprint' frame to the 'scan' frame.
    """
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.frame_id = "base_footprint"
    transform.child_frame_id = "scan"
    transform.transform.translation.x = -0.2
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    transform.header.stamp = current_time
    tf_broadcaster.sendTransform(transform)

def publish_origin_to_map_transform(simulator, tf_broadcaster, current_time):
    """
    Publishes the transformation from the 'origin' frame to the 'map' frame.
    """
    transform = geometry_msgs.msg.TransformStamped()

    # Set the frame IDs
    transform.header.frame_id = "origin"
    transform.child_frame_id = "map"
    
    bounds = simulator.pathfinder.get_bounds()


    #TODO: adjust bounds
    # Set the translation (this depends on where the scanner is mounted on the robot)
    transform.transform.translation.x = -(bounds[1][0]-bounds[0][0])/2#-9.16  # upperbound - lower bound /2 distance forward from the base_link
    transform.transform.translation.y = -(bounds[1][2]-bounds[0][2])/2# -12.31  # distance left from the base_link
    transform.transform.translation.z = 0.0  # height above the base_link

    # Set the rotation (this depends on the orientation of the scanner)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)  # no rotation
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    # Set the timestamp
    transform.header.stamp = current_time

    # Publish the transform
    tf_broadcaster.sendTransform(transform)

