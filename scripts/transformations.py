import rospy
import numpy as np
import geometry_msgs.msg
import tf

def publish_transforms(agent_state, tf_broadcaster):
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
    transform.header.stamp = rospy.Time.now()

    # Publish the transform
    tf_broadcaster.sendTransform(transform)

def publish_noisy_odom_transform(agent_state, tf_broadcaster, noise_std_dev=0.1):
    transform = geometry_msgs.msg.TransformStamped()

    # Set the frame IDs
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"

    # Set the translation with added noise
    transform.transform.translation.x = agent_state.position[0] + np.random.normal(0, noise_std_dev)
    transform.transform.translation.y = agent_state.position[1] + np.random.normal(0, noise_std_dev)
    transform.transform.translation.z = agent_state.position[2] + np.random.normal(0, noise_std_dev)

    # Set the rotation with added noise
    quaternion = (agent_state.rotation.x, agent_state.rotation.y, agent_state.rotation.z, agent_state.rotation.w)
    euler_angles = tf.transformations.euler_from_quaternion(quaternion)
    noisy_euler_angles = [angle + np.random.normal(0, noise_std_dev) for angle in euler_angles]
    noisy_quaternion = tf.transformations.quaternion_from_euler(*noisy_euler_angles)
    transform.transform.rotation.x = noisy_quaternion[0]
    transform.transform.rotation.y = noisy_quaternion[1]
    transform.transform.rotation.z = noisy_quaternion[2]
    transform.transform.rotation.w = noisy_quaternion[3]

    # Set the timestamp
    transform.header.stamp = rospy.Time.now()

    # Publish the transform
    tf_broadcaster.sendTransform(transform)

def publish_map_odom_transform(agent_state, tf_broadcaster, noise_std_dev=0.1):
    transform = geometry_msgs.msg.TransformStamped()

    # Set the frame IDs
    transform.header.frame_id = "map"
    transform.child_frame_id = "odom"

    # Set the translation with added noise
    transform.transform.translation.x = agent_state.position[0] + np.random.normal(0, noise_std_dev)
    transform.transform.translation.y = agent_state.position[1] + np.random.normal(0, noise_std_dev)
    transform.transform.translation.z = agent_state.position[2] + np.random.normal(0, noise_std_dev)

    # Set the rotation with added noise
    quaternion = (agent_state.rotation.x, agent_state.rotation.y, agent_state.rotation.z, agent_state.rotation.w)
    euler_angles = tf.transformations.euler_from_quaternion(quaternion)
    noisy_euler_angles = [angle + np.random.normal(0, noise_std_dev) for angle in euler_angles]
    noisy_quaternion = tf.transformations.quaternion_from_euler(*noisy_euler_angles)
    transform.transform.rotation.x = noisy_quaternion[0]
    transform.transform.rotation.y = noisy_quaternion[1]
    transform.transform.rotation.z = noisy_quaternion[2]
    transform.transform.rotation.w = noisy_quaternion[3]

    # Set the timestamp
    transform.header.stamp = rospy.Time.now()

    # Publish the transform
    tf_broadcaster.sendTransform(transform)
    
def publish_base_link_to_scan_transform(tf_broadcaster):
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
    transform.header.stamp = rospy.Time.now()

    # Publish the transform
    tf_broadcaster.sendTransform(transform)

