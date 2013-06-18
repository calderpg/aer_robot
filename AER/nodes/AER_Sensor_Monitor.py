#!/usr/bin/python

from PlatformComponents import *

import roslib; roslib.load_manifest('AER')
import rospy

from AER.msg import Sensor
from AER.msg import Bumper


class SensorMonitor():

    def __init__(self):
        rospy.init_node('Sensor_Monitor')
        port = rospy.get_param('Sensor_Monitor/sensor_port')
        self.micro = AERsensor(port, 115200, "Sensor platform")
        self.bumper_pub = rospy.Publisher("aer_bumpers", Bumper)
        self.sensor_pub = rospy.Publisher("aer_sensors", Sensor)
        rate = rospy.Rate(rospy.get_param('~hz', 60))
        
        while not rospy.is_shutdown():
            rate.sleep()
            sensor_message = Sensor()
            bumper_message = Bumper()
            sensor_state = self.micro.GetState()
            bumper_message.bumper1 = sensor_state[1].RFF
            bumper_message.bumper2 = sensor_state[1].RFS
            bumper_message.bumper3 = sensor_state[1].RRS
            bumper_message.bumper4 = sensor_state[1].RRB
            bumper_message.bumper5 = sensor_state[1].LRB
            bumper_message.bumper6 = sensor_state[1].LRS
            bumper_message.bumper7 = sensor_state[1].LFS
            bumper_message.bumper8 = sensor_state[1].LFF
            sensor_message.accelerometer.x = sensor_state[0].X_A
            sensor_message.accelerometer.y = sensor_state[0].Y_A
            sensor_message.accelerometer.z = sensor_state[0].Z_A
            sensor_message.gyroscope.z = sensor_state[0].Z_R
            self.bumper_pub.publish(bumper_message)
            self.sensor_pub.publish(sensor_message)


if __name__ == "__main__":
    SensorMonitor()
