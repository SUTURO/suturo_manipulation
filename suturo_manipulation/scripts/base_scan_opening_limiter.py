#! /usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan

class BaseScanLimiterServer:
    IGNORE_AREA = 30
    laser_scan_publisher = rospy.Publisher('hsrb/test_scan', LaserScan)

    def __init__(self, name):
        self._action_name = name
        laser_scan_subscriber = rospy.Subscriber('/hsrb/base_scan_unlimited', LaserScan, self.laser_scan_callback)


    def laser_scan_callback(self, msg):
        sensor_ranges = np.array(msg.ranges)
        sensor_middle = len(sensor_ranges) / 2
        sensor_ranges[(sensor_middle - (self.IGNORE_AREA * 3)):sensor_middle + (self.IGNORE_AREA * 3)] =\
            [float('inf')] * (self.IGNORE_AREA * 2 * 3)
        msg.ranges = tuple(sensor_ranges)
        self.laser_scan_publisher.publish(msg)


if __name__ == '__main__':
    rospy.init_node('base_scan_limiter')
    server = BaseScanLimiterServer(rospy.get_name())
    rospy.spin()