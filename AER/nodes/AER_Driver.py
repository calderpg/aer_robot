#!/usr/bin/python

from PlatformComponents import *
from DriveControllers import *
from TestComponents import *

import roslib; roslib.load_manifest('AER')
import rospy

from AER.msg import Motor
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class AERdriver:
    def __init__(self, real):
        rospy.init_node('AER_Driver')
        if real:
            port = rospy.get_param('AER_Driver/control_port')
            self.robot = AERplatform(port, 115200, 'platform')
        else:
            self.robot = EMUplatform("/dev/emulator", 230400, 'emulator')
        
        self.controller = MecanumDrive()

        rospy.Subscriber("cmd_vel", Twist, self.callback)
        self.state_pub = rospy.Publisher("aer_motors", Motor)
        rate = rospy.Rate(rospy.get_param('~hz', 60))
        
        while not rospy.is_shutdown():
            rate.sleep()
            latest_state = self.robot.GetState()
            motor_message = Motor()
            motor_message.front.M0 = latest_state.LF
            motor_message.front.M1 = latest_state.RF
            motor_message.rear.M0 = latest_state.LR
            motor_message.rear.M1 = latest_state.RR
            self.state_pub.publish(motor_message)

    def callback(self, data):
        """Send Twist commands to the robot"""
        X = data.linear.x
        Y = data.linear.y
        Z = data.angular.z
        #print "X : " + str(X) + " Y : " + str(Y) + " Z : " + str(Z)
        commands = self.controller.Compute(X, Y, Z)
        self.robot.Go(commands[0], commands[1], commands[2], commands[3])


if __name__ == "__main__":
    AERdriver(True)
