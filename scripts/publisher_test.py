#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
import numpy as np
import habitat
# from habitat.config.default import get_config
import random
from habitat.core.simulator import AgentState
from HabitatRosIntegration.msg import BasicAction
import sys, termios, tty, os


def talker():
    pub = rospy.Publisher('eval_ready', BasicAction, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        move_cmd = BasicAction()
        move_cmd.Action = "STOP"
        move_cmd.ActionIdx = 0

        # read a single key from the user
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        # check if the key is entered
        if ch:
            if ch in ['0', '1', '2', '3', '4']:
                move_cmd.ActionIdx = int(ch)
                move_cmd.Action = ["STOP", "FORWARD", "TURN LEFT", "TURN RIGHT", "PRINT SCREEN"][move_cmd.ActionIdx]
                rospy.loginfo(move_cmd)
                pub.publish(move_cmd)
            else:
                rospy.loginfo("Action not defined")

        rate.sleep()

if __name__ == '__main__':
    # print("start config")
    # config = habitat.get_config(
    #     config_path="/home/aaron/catkin_ws/src/HabitatRosIntegration/src/config/website_config.yaml"
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
