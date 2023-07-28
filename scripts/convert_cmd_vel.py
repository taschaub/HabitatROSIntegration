#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from publish_test.msg import BasicAction
import time

# Define actions
ACTIONS = ["STOP", "FORWARD", "TURN LEFT", "TURN RIGHT", "PRINT SCREEN"]

# Define publishing frequency (in Hz)
PUBLISH_FREQ = 2  # You can modify this to set the desired frequency

# Angular velocity tolerance (change this value as per your requirements)
ANGULAR_VEL_TOLERANCE = 0.1# Arbitrary value, should be tuned for your specific case

# Last publishing time
last_publish_time = time.time()

def cmd_vel_callback(data):
    global pub, last_publish_time, PUBLISH_FREQ, ANGULAR_VEL_TOLERANCE
    current_time = time.time()

    # Check if it's time to publish based on the frequency
    if current_time - last_publish_time < 1.0 / PUBLISH_FREQ:
        return
    last_publish_time = current_time

    linear_vel = data.linear.x
    angular_vel = data.angular.z

    move_cmd = BasicAction()

    # Check angular velocity first. If it's greater than the tolerance, adjust the heading.
    if abs(angular_vel) > ANGULAR_VEL_TOLERANCE:
        print("angl vel:" + str(angular_vel))
        if angular_vel > 0:
            move_cmd.ActionIdx = 2  # TURN LEFT
        else:
            move_cmd.ActionIdx = 3  # TURN RIGHT
    # If we're already heading in the right direction (within the tolerance), move forward.
    elif linear_vel > 0:
        move_cmd.ActionIdx = 1  # FORWARD
    else:
        return  # Do nothing if the action would be STOP

    move_cmd.Action = ACTIONS[move_cmd.ActionIdx]
    rospy.loginfo(move_cmd)
    pub.publish(move_cmd)

def talker():
    global pub
    pub = rospy.Publisher('chatter', BasicAction, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    
    # Subscribe to cmd_vel topic
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

    # Spin to keep the script alive
    rospy.spin()

if __name__ == "__main__":
    talker()
