#!/usr/bin/env python

import sys
import roslib; roslib.load_manifest('AER')
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Teleop:
    def __init__(self):
        rospy.init_node('AER_teleop')

        self.x_axis = 1 #number of up-down axis on left analog stick
        self.y_axis = 0 #number of left-right axis on left analog stick
        self.left_trigger = 2 #number of left trigger axis
        self.right_trigger = 5 #number of right trigger axis
        self.deadman_button = 0 #number of green button
        self.last_analog_X = 0.0
        self.last_analog_Y = 0.0
        self.last_trigger_l = 0.0
        self.last_trigger_r = 0.0
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist)

        rospy.Subscriber("joy", Joy, self.NewState)
        rate = rospy.Rate(rospy.get_param('~hz', 60))
        
        while not rospy.is_shutdown():
            rate.sleep()
            cleaned = self.Normalize(self.last_analog_X, -self.last_analog_Y, self.last_trigger_l, self.last_trigger_r)
            self.Publish(cleaned)
                

    def Normalize(self, X, Y, Zleft, Zright):
        """ Converts raw joystick axis values into linear and angular movement commands """
        normalized_left_trigger = -((Zleft - 1.0) / 2.0)
        normalized_right_trigger = ((Zright - 1.0) / 2.0)
        Z = normalized_left_trigger + normalized_right_trigger
        print "Received:\nZ L: " + str(Zleft) + " Z R: " + str(Zright) + "\nProduced:\nZ : " + str(Z)
        temp_x = 0.0
        temp_y = 0.0
        temp_z = 0.0
        max_linear = 1.5
        min_linear = 0.3
        max_angular = 2.4
        min_angular = 0.5
        dead_zone = 0.05
        max_axis = 1.0
        temp_x = X * 1.2
        if X > 0.0:
            temp_x = temp_x + 0.3
        elif X < -0.0:
            temp_x = temp_x - 0.3
        temp_y = Y * 1.2
        if Y > 0.0:
            temp_y = temp_y + 0.3
        elif Y < -0.0:
            temp_y = temp_y - 0.3
        temp_z = Z * 1.9
        if Z > 0.0:
            temp_z = temp_z + 0.5
        elif Z < -0.0:
            temp_z = temp_z - 0.5
        temp_z = temp_z
        #safety check
        #print "And produced:\nX : " + str(temp_x) + " Y : " + str(temp_y) + " Z : " + str(temp_z)
        return [temp_x, temp_y, temp_z]

    def Publish(self, cleaned):
        """Take normalized data, make Twist message. """
        cmd = Twist()
        cmd.linear.x = cleaned[0]
        cmd.linear.y = cleaned[1]
        cmd.angular.z = cleaned[2]
        self.cmd_pub.publish(cmd)

    def NewState(self, data):
        """Receive joystick data, save it in the state storage variables"""
        if data.buttons[self.deadman_button] == 1:
            self.last_analog_X = data.axes[self.x_axis]
            self.last_analog_Y = data.axes[self.y_axis]
            self.last_trigger_l = data.axes[self.left_trigger]
            self.last_trigger_r = data.axes[self.right_trigger]
        else:
            self.last_analog_X = 0.0
            self.last_analog_Y = 0.0
            self.last_trigger_l = 0.0
            self.last_trigger_r = 0.0


if __name__ == "__main__":
    Teleop()
