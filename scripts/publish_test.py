#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
import numpy as np
import habitat
# from habitat.config.default import get_config
import random
from habitat.core.simulator import AgentState
from geometry_msgs.msg import Twist


def talker():
    pub = rospy.Publisher('chatter', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        move_cmd = Twist()
        move_cmd.linear.x = 1.0
        rospy.loginfo(move_cmd)
        pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    # print("start config")
    # config = habitat.get_config(
    #     config_path="/home/aaron/catkin_ws/src/publish_test/src/config/website_config.yaml"
    # )
    # print("Environment creation successful")
    # print(config)

    # env = habitat.Env(config=config)
    # print("env node initialised")

    # with habitat.Env(
    #     config=habitat.get_config()
    # ) as env:
    #     print("Environment creation successful")
    #     observations = env.reset()  # noqa: F841

    #     print("Agent acting inside environment.")
    #     count_steps = 0
    #     while not env.episode_over:
    #         observations = env.step(env.action_space.sample())  # noqa: F841
    #         count_steps += 1
    #     print("Episode finished after {} steps.".format(count_steps))

    # action_mapping = {
    #     0: 'stop',
    #     1: 'move_forward',
    #     2: 'turn left',
    #     3: 'turn right'
    # }

    # max_steps = 6

    # env.episodes = random.sample(env.episodes, 2)

    # for i in range(len(env.episodes)):
    #     observations = env.reset()
    #     count_steps = 0
    #     while count_steps < max_steps:
    #         print(AgentState.position)
            # action = random.choice(list(action_mapping.keys()))
    #         print(action_mapping[action])
    #         observations = env.step(action)
    #         count_steps += 1
    #     if env.episode_over:
    #         break

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
