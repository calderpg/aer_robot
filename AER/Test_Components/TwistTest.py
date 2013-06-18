#!/usr/bin/env python

import sys
import random
import roslib; roslib.load_manifest('AER')
import rospy

from geometry_msgs.msg import Twist


class TwistTest:
    def __init__(self):
        rospy.init_node('AER_teleop')
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist)
        rate = rospy.Rate(rospy.get_param('~hz', 1))
        
        while not rospy.is_shutdown():
            rate.sleep()
            r1 = random.uniform(-1.5, 1.5)
            r2 = random.uniform(-1.0, 1.0)
            r3 = random.uniform(-2.0, 2.0)
            print "X: " + str(r1) + " Y: " + str(r2) + " Z: " + str(r3)
            cleaned = [r1, r2, r3]
            self.Publish(cleaned)

    def Publish(self, cleaned):
        """Take normalized data, make Twist message. """
        cmd = Twist()
        cmd.linear.x = cleaned[0]
        cmd.linear.y = cleaned[1]
        cmd.angular.z = cleaned[2]
        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    TwistTest()
