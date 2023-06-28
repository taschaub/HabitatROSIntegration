import rospy
import numpy as np
import geometry_msgs.msg
import tf

def habitat_to_ros_quaternion(agent_rot):
    # Quaternion that represents a 90 degrees rotation around Z in ROS (right-handed, so it's clockwise)
    transform_quat_ros = tf.transformations.quaternion_from_euler(0, 0, np.pi / 2,)

    # Convert Habitat quaternion to ROS quaternion (both are [x, y, z, w])
    agent_quat_ros = tf.transformations.quaternion_multiply(
        [agent_rot.x, -agent_rot.z, agent_rot.y, agent_rot.w], transform_quat_ros
    )
    
    return agent_quat_ros


def publish_transforms(agent_state, tf_broadcaster, current_time):
    transform = geometry_msgs.msg.TransformStamped()

    # Set the frame IDs
    transform.header.frame_id = "map"
    transform.child_frame_id = "base_link"

    # Set the translation
    transform.transform.translation.x = agent_state.position[0]
    transform.transform.translation.y = agent_state.position[1]
    transform.transform.translation.z = agent_state.position[2]

    # Set the rotation
    quaternion = (agent_state.rotation.x, agent_state.rotation.y, agent_state.rotation.z, agent_state.rotation.w)
    euler_angles = tf.transformations.euler_from_quaternion(quaternion)
    quaternion = tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    # Set the timestamp
    transform.header.stamp = current_time

    # Publish the transform
    tf_broadcaster.sendTransform(transform)

def publish_odom_baselink_transform(agent_state, tf_broadcaster, current_time,noise_std_dev=0.1):
    transform = geometry_msgs.msg.TransformStamped()

    # Set the frame IDs
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"
    
    agent_pos = agent_state.position
    agent_rot = agent_state.rotation

    # Set the translation with added noise
    transform.transform.translation.x = agent_pos[0]
    transform.transform.translation.y = -agent_pos[2]
    transform.transform.translation.z = agent_pos[1]
    
    quaternion = habitat_to_ros_quaternion(agent_rot)
    transform.transform.rotation.x = quaternion[0] 
    transform.transform.rotation.y = quaternion[1] 
    transform.transform.rotation.z = quaternion[2] 
    transform.transform.rotation.w = quaternion[3] 

    # Set the timestamp
    transform.header.stamp = current_time

    # Publish the transform
    tf_broadcaster.sendTransform(transform)

def publish_map_odom_transform(env, tf_broadcaster, current_time, noise_std_dev=0.1):
    transform = geometry_msgs.msg.TransformStamped()

    # Set the frame IDs
    transform.header.frame_id = "map"
    transform.child_frame_id = "odom"
    bounds = env.sim.pathfinder.get_bounds()
        
    pos =[-bounds[0][0],bounds[1][2],0] #lowerbound x , upper y

    # Set the translation with added noise
    transform.transform.translation.x = pos[0]
    transform.transform.translation.y = pos[1]
    transform.transform.translation.z = pos[2]
    # Set the rotation
    #quaternion = (agent_state.rotation.x, agent_state.rotation.y, agent_state.rotation.z, agent_state.rotation.w)
    #euler_angles = tf.transformations.euler_from_quaternion(quaternion)
    #quaternion = tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)  # no rotation

    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    # Set the timestamp
    transform.header.stamp = current_time

    # Publish the transform
    tf_broadcaster.sendTransform(transform)
    
def publish_base_link_to_scan_transform(tf_broadcaster, current_time):
    transform = geometry_msgs.msg.TransformStamped()

    # Set the frame IDs
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "scan"

    # Set the translation (this depends on where the scanner is mounted on the robot)
    transform.transform.translation.x = 0.0  # distance forward from the base_link
    transform.transform.translation.y = 0.0  # distance left from the base_link
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
    
def publish_origin_to_map_transform(tf_broadcaster, current_time):
    transform = geometry_msgs.msg.TransformStamped()

    # Set the frame IDs
    transform.header.frame_id = "origin"
    transform.child_frame_id = "map"

    # Set the translation (this depends on where the scanner is mounted on the robot)
    transform.transform.translation.x = -9.16  # upperbound - lower bound /2 distance forward from the base_link
    transform.transform.translation.y = -12.31  # distance left from the base_link
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

