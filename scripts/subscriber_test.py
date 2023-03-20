#!/usr/bin/env python
import rospy
from publish_test.msg import BasicAction

action_mapping = {
        0: 'stop',
        1: 'move_forward',
        2: 'turn left',
        3: 'turn right'
    }

def callback(data):
    receivedAction=action_mapping[data.ActionIdx]
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.Action)
    print(receivedAction)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", BasicAction, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()