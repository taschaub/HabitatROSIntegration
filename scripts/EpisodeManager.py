import json
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import String
from publish_test.msg import SetupHabitat, RunTime

class EpisodeManager:
    def __init__(self):
        # Load episode data from JSON file
        with open('episodes.json') as f:
            self.episodes = json.load(f)

        # Initialize current episode
        self.current_episode = self.episodes[0]
        self.episode_id = 0

        # Initialize ROS publishers and subscribers
        self.setup_pub = rospy.Publisher('/setup_habitat', SetupHabitat, queue_size=10)
        self.run_time_sub = rospy.Subscriber('/run_time', RunTime, self.run_time_callback)

        # Initialize SetupHabitat message
        self.setup_msg = SetupHabitat()

    def run_time_callback(self, msg):
        # If run_time is end time, move to the next episode
        if msg.run_time_type == 'end':
            self.episode_id += 1
            if self.episode_id < len(self.episodes):
                self.current_episode = self.episodes[self.episode_id]
                self.send_setup_msg()
            else:
                rospy.loginfo("All episodes completed.")

    def send_setup_msg(self):
        # Fill SetupHabitat message with current episode data
        self.setup_msg.SceneName = self.current_episode['scene']
        self.setup_msg.StartPoint.position = Point(**self.current_episode['start']['position'])
        self.setup_msg.StartPoint.orientation = Quaternion(**self.current_episode['start']['orientation'])
        self.setup_msg.GoalPoint.position = Point(**self.current_episode['goal']['position'])
        self.setup_msg.GoalPoint.orientation = Quaternion(**self.current_episode['goal']['orientation'])

        # Publish SetupHabitat message
        self.setup_pub.publish(self.setup_msg)

if __name__ == '__main__':
    rospy.init_node('episode_manager_node')
    em = EpisodeManager()
    em.send_setup_msg()  # Send initial setup message
    rospy.spin()
