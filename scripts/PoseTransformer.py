#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

class PoseTransformer:
    def __init__(self):
        self.listener = tf.TransformListener()

    def transform_pose(self, input_pose, target_frame):
        """
        Transforms the given pose to the specified frame.

        :param input_pose: geometry_msgs/PoseStamped - The pose you want to transform
        :param target_frame: str - The target frame you want to transform to
        :return: geometry_msgs/PoseStamped - The transformed pose
        """
        try:
            # Wait for the transform to be available
            self.listener.waitForTransform(target_frame, input_pose.header.frame_id, rospy.Time(), rospy.Duration(4.0))

            # Transform the pose
            transformed_pose = self.listener.transformPose(target_frame, input_pose)
            
            return transformed_pose

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF Exception during transform: %s", e)
            return None

if __name__ == '__main__':
    rospy.init_node('pose_transformer_test')

    transformer = PoseTransformer()

    # Example PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.0
    pose.pose.position.y = 2.0
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0  # Assume facing forward for simplicity

    transformed_pose = transformer.transform_pose(pose, "odom")
    if transformed_pose:
        rospy.loginfo("Transformed Pose: %s", transformed_pose)
