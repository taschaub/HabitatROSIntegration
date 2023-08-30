#!/usr/bin/env python3

import rospy
import tf
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from publish_test.msg import BasicAction
import pandas as pd
import pickle
import os

class Evaluation:
    base_directory = '/home/aaron/catkin_ws/src/publish_test/evaluation/evaluation_rl02/'  # Class variable for the base directory
    
    def __init__(self):
        # Initialize node
        rospy.init_node('evaluation')
        
        # Initialize tf listener
        self.listener = tf.TransformListener()

        # Initialize subscribers
        rospy.Subscriber("move_base/status", GoalStatusArray, self.move_base_status_callback)
        rospy.Subscriber("crash_detect",  BasicAction, self.collision_callback)
        self.run_time_pub = rospy.Publisher('run_time', BasicAction, queue_size=10)

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
                self.jump_to_next_episode()
                
            elif status == 4 and self.goal_active:  # If the goal is aborted
                print("goal aborted")
                self.goal_active = False
                self.episodes[self.current_episode_id]['end_time'] = rospy.Time.now().to_sec()
                self.episodes[self.current_episode_id]['aborted'] = True  # Add this line to record the aborted status
                self.jump_to_next_episode()

    def get_transform(self):
        # This method will get the transform from odom to base_link
        try:
            (trans, rot) = self.listener.lookupTransform('odom', 'base_link', rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None, None
        
    def record_position(self):
        
        # If there's a goal active, store the robot's position in the path
        if self.goal_active:
            trans, rot = self.get_transform()
            if trans and rot:
                if 'path' not in self.episodes[self.current_episode_id]:
                    self.episodes[self.current_episode_id]['path'] = []
                # Save just the translation for now, but you can modify to save rotation as well if required.
                self.episodes[self.current_episode_id]['path'].append(trans)


        #create copy so that i dont get runtime error "changed size during interation"
        episodes_copy = dict(self.episodes)
        
        if not os.path.exists(Evaluation.base_directory):  # Step 2: Check if the directory exists
            os.makedirs(Evaluation.base_directory)

        # Save path data for each episode (you can modify the way it's saved based on your needs)
        for episode_id, data in episodes_copy.items():
            if 'path' in data:
                with open(f'{Evaluation.base_directory}path_data_episode_{episode_id}.csv', 'w') as f:
                    for pos in data['path']:
                        f.write(f"{pos[0]},{pos[1]}\n")
    
    
    def collision_callback(self, msg):
        # Increment current episode's collision count
        if self.current_episode_id in self.episodes:
            self.episodes[self.current_episode_id]['collisions'] += 1

    def save_data(self):
        # Create a DataFrame from self.episodes
        df = pd.DataFrame.from_dict(self.episodes, orient='index')

        # Save DataFrame to a CSV file
        df.to_csv('{Evaluation.base_directory}evaluation_data.csv')

    def jump_to_next_episode(self):
        move_cmd = BasicAction()
        move_cmd.Action = "STOP"
        move_cmd.ActionIdx = 0
        rospy.loginfo(move_cmd)
        self.run_time_pub.publish(move_cmd)
        
        
    def compute_metrics(self):
        # TODO: Compute metrics based on self.episodes
        pass

if __name__ == "__main__":
    evaluation = Evaluation()
    
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        evaluation.record_position()
        rate.sleep()
        
    evaluation.save_data()  #