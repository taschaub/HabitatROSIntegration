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
import sys, termios, tty, os


def talker():
    pub = rospy.Publisher('chatter', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        move_cmd = Twist()
        move_cmd.linear.x = 1.0
        
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
            rospy.loginfo(move_cmd)
            pub.publish(move_cmd)
        
        rate.sleep()


if __name__ == '__main__':
 
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
