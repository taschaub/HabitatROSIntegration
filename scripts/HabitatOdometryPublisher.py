import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
from habitat.sims.habitat_simulator import HabitatSim

class HabitatOdometryPublisher:

    def __init__(self, initial_pose, action_duration):
        self.current_pose = initial_pose
        self.previous_pose = initial_pose
        self.action_duration = action_duration
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

    def update_pose(self, new_pose):
        # Compute odometry
        delta_x = new_pose.position.x - self.previous_pose.position.x
        delta_y = new_pose.position.y - self.previous_pose.position.y
        delta_th = new_pose.rotation.yaw - self.previous_pose.rotation.yaw

        vx = delta_x / self.action_duration
        vy = delta_y / self.action_duration
        vth = delta_th / self.action_duration

        # Construct odometry message
        current_time = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set position and orientation
        odom.pose.pose = Pose(Point(new_pose.position.x, new_pose.position.y, 0.), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, new_pose.rotation.yaw)))
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # Publish odometry message
        self.odom_pub.publish(odom)

        # Publish transform
        self.odom_broadcaster.sendTransform(
            (new_pose.position.x, new_pose.position.y, 0.),
            tf.transformations.quaternion_from_euler(0, 0, new_pose.rotation.yaw),
            current_time,
            "base_link",
            "odom"
        )

        # Update previous pose
        self.previous_pose = new_pose

    def run(self):
        while not rospy.is_shutdown():
            # Here, you would get the new pose from the Habitat simulator.
            # In this example, we'll just keep the pose constant.
            new_pose = HabitatSim.AgentState(position=self.current_pose.position, rotation=self.current_pose.rotation)
            self.update_pose(new_pose)
