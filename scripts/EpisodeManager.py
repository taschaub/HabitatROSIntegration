import json
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from nav_msgs.msg import Path
from std_msgs.msg import String
from HabitatRosIntegration.msg import SetupHabitat, BasicAction

class PoseTransformer:
    def __init__(self):
        self.listener = tf.TransformListener()

    def transform_pose(self, pose, from_frame="map", to_frame="odom"):
        """Transforms a pose to the specified target frame."""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = from_frame
        pose_stamped.pose = pose
        try:
            self.listener.waitForTransform(to_frame, from_frame, rospy.Time(), rospy.Duration(4.0))
            transformed_pose = self.listener.transformPose(to_frame, pose_stamped)
            return transformed_pose.pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF Exception during transform: %s", e)
            return None

class EpisodeManager:
    def __init__(self):
        # Initialize PoseTransformer
        rospy.sleep(1)  # Delay for 1 second so that first message is sent

        self.transformer = PoseTransformer()
        
        # Load episode data from JSON file
        with open('/home/aaron/catkin_ws/src/HabitatRosIntegration/episodes/normal_rotated.json') as f: # combined_episodes_inverted_again
            self.episodes = json.load(f)

        # Initialize current episode
        self.current_episode = self.episodes[0]
        self.episode_id = 0

        # Initialize ROS publishers and subscribers
        self.setup_pub = rospy.Publisher('setup_habitat', SetupHabitat, queue_size=10)
        self.eval_ready_sub = rospy.Subscriber('eval_ready', BasicAction, self.eval_ready_callback)

        # Initialize SetupHabitat message
        self.setup_msg = SetupHabitat()

    def eval_ready_callback(self, msg):
        # If eval_ready is end time, move to the next episode
        if msg.ActionIdx == 0:
            self.episode_id += 1
            if self.episode_id < len(self.episodes):
                self.current_episode = self.episodes[self.episode_id]
                self.send_setup_msg()
            else:
                rospy.loginfo("All episodes completed.")

    def send_setup_msg(self):
        # Transform start and goal positions to odom frame
        start_pose = Pose(Point(**self.current_episode['start']['position']),
                          Quaternion(**self.current_episode['start']['orientation']))
        goal_pose = Pose(Point(**self.current_episode['goal']['position']),
                         Quaternion(**self.current_episode['goal']['orientation']))
        
        start_pose_transformed = self.transformer.transform_pose(start_pose)
        
        #dont transform goal pose
        # goal_pose_transformed = self.transformer.transform_pose(goal_pose)

        # Fill SetupHabitat message with transformed episode data
        # self.setup_msg.EpisodeId = self.current_episode['episode']
        self.setup_msg.SceneName = self.current_episode['scene']
        
        self.setup_msg.StartPoint = start_pose #_transformed
        self.setup_msg.GoalPoint = goal_pose #_transformed

        # Publish SetupHabitat message
        self.setup_pub.publish(self.setup_msg)

if __name__ == '__main__':
    rospy.init_node('episode_manager_node')
    em = EpisodeManager()
    rospy.sleep(1)  # Delay for 1 second so that first message is sent

    em.send_setup_msg()  # Send initial setup message
    rospy.spin()
