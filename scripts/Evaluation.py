#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from publish_test.msg import NavigationRun
from geometry_msgs.msg import PoseStamped
import pandas as pd
import pickle

class Evaluation:
    def __init__(self):
        # Initialize node
        rospy.init_node('evaluation')

        # Initialize subscribers
        rospy.Subscriber("path_topic", Path, self.path_callback)
        rospy.Subscriber("nav_run_topic", NavigationRun, self.nav_run_callback)

        # Initialize data structures
        self.episodes = {}  # Each episode's data is stored in a dict with keys 'path', 'run_time', 'collisions'

    def path_callback(self, msg):
        # Append current pose to current episode's path
        current_episode = self.episodes[self.current_episode_id]
        for pose in msg.poses:
            current_episode['path'].append(pose)

    def nav_run_callback(self, msg):
        # If it's the start of a new episode, initialize a new dict in self.episodes
        if msg.start_time > msg.end_time:
            self.current_episode_id = msg.episode_id
            self.episodes[msg.episode_id] = {
                'path': [],
                'run_time': [msg.start_time],
                'collisions': 0
            }

        # If it's the end of an episode, append the end time to the current episode's run time
        else:
            self.episodes[msg.episode_id]['run_time'].append(msg.end_time)

    def collision_callback(self, msg):
        # Increment current episode's collision count
        self.episodes[self.current_episode_id]['collisions'] += 1

    def save_data(self):
        # Create a DataFrame from self.episodes
        df = pd.DataFrame.from_dict(self.episodes, orient='index')

        # Convert 'path' column to serialized strings
        #df['path'] = df['path'].apply(pickle.dumps)
        df['run_time'] = df['run_time'].apply(lambda times: [time.to_sec() for time in times])

        # Save DataFrame to a CSV file
        df.to_csv('evaluation_data.csv')

    def compute_metrics(self):
        # TODO: Compute metrics based on self.episodes
        pass
