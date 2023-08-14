#!/usr/bin/env python3

import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from publish_test.msg import BasicAction
import pandas as pd
import pickle

class Evaluation:
    def __init__(self):
        # Initialize node
        rospy.init_node('evaluation')

        # Initialize subscribers
        rospy.Subscriber("move_base/status", GoalStatusArray, self.move_base_status_callback)
        rospy.Subscriber("crash_detect",  BasicAction, self.collision_callback)

        # Initialize data structures
        self.current_episode_id = 0  # increment every time a new goal is set
        self.episodes = {}  # Each episode's data is stored in a dict with keys 'start_time', 'end_time', 'collisions'
        
        self.goal_active = False

    def move_base_status_callback(self, msg):
        # Check the latest status message
        if len(msg.status_list) > 0:
            status = msg.status_list[-1].status

            # New goal has been published
            if status == 1 and not self.goal_active:
                print("goal detected")
                self.goal_active = True
                self.current_episode_id += 1
                self.episodes[self.current_episode_id] = {
                    'start_time': rospy.Time.now().to_sec(),
                    'end_time': None,
                    'collisions': 0
                }

            # Goal is reached (succeeded)
            elif status == 3 and self.goal_active:
                print("goal reached")
                self.goal_active = False
                self.episodes[self.current_episode_id]['end_time'] = rospy.Time.now().to_sec()

    def collision_callback(self, msg):
        # Increment current episode's collision count
        if self.current_episode_id in self.episodes:
            self.episodes[self.current_episode_id]['collisions'] += 1

    def save_data(self):
        # Create a DataFrame from self.episodes
        df = pd.DataFrame.from_dict(self.episodes, orient='index')

        # Save DataFrame to a CSV file
        df.to_csv('/home/aaron/catkin_ws/src/publish_test/scripts/evaluation_data.csv')

    def compute_metrics(self):
        # TODO: Compute metrics based on self.episodes
        pass

if __name__ == "__main__":
    evaluation = Evaluation()
    rospy.spin()
    evaluation.save_data()  # save data when the node is shutdown
