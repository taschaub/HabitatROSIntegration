#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserScanFixer:
    def __init__(self):
        rospy.init_node('laser_scan_fixer', anonymous=True)
        self.pub = rospy.Publisher('fixed_scan', LaserScan, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.callback)

    def callback(self, data):
        # Reverse the order of the range measurements
        data.ranges = list(reversed(data.ranges))

        # Change 'nan' to 'inf'
        data.ranges = [float('inf') if np.isnan(x) else x for x in data.ranges]

        # Add intensities with 0.0
        data.intensities = [0.0]*len(data.ranges)

        self.pub.publish(data)

if __name__ == '__main__':
    try:
        lsf = LaserScanFixer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
