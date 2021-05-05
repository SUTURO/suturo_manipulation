import rospy

class BaseScanLimiterServer:

    def __init__(self, name):
        self.

if __name__ == '__main__':
    rospy.init_node('base_scan_limiter')
    server = BaseScanLimiterServer(rospy.get_name())
    rospy.spin()