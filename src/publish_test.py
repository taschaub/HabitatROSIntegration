#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import numpy as np
import habitat
from habitat.config.default import get_config  
import random
from habitat.core.simulator import AgentState


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    
    config = get_config(config_path="config/test_config.yaml")
    print(config)
    env = habitat.Env(config=config)

    action_mapping = {
        0: 'stop',
        1: 'move_forward',
        2: 'turn left',
        3: 'turn right'
    }
    
    max_steps = 6
    
    env.episodes = random.sample(env.episodes, 2)
    
    for i in range(len(env.episodes)):
        observations = env.reset()
        count_steps = 0
        while count_steps < max_steps:
            print(AgentState.position)
            action = random.choice(list(action_mapping.keys()))
            print(action_mapping[action])
            observations = env.step(action)
            count_steps += 1
        if env.episode_over:
            break
    
    
    
    
    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass