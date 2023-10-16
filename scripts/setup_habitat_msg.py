#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
import numpy as np
import habitat
# from habitat.config.default import get_config
import random
from habitat.core.simulator import AgentState
from HabitatRosIntegration.msg import SetupHabitat
import sys, termios, tty, os
from geometry_msgs.msg import Point, Quaternion



def talker():
    pub = rospy.Publisher('setup_habitat', SetupHabitat, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        setup_msg = SetupHabitat()
        setup_msg.SceneName = "TestName"
        setup_msg.StartPoint.position = Point(1,0,0)
        setup_msg.StartPoint.orientation = Quaternion(1,0,0,0)
        setup_msg.GoalPoint = setup_msg.StartPoint

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
                setup_msg.StartPoint.position = Point(2,0,0)
                rospy.loginfo(setup_msg)
                pub.publish(setup_msg)
            else:
                rospy.loginfo("Action not defined")

        rate.sleep()

if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
