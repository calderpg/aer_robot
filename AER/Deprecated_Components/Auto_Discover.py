#!/usr/bin/python

import serial
import glob
import subprocess

import roslib; roslib.load_manifest('AER')
import rospy

def identify():
    micros = glob.glob('/dev/ttyACM*')
    assert len(micros) > 1
    micro1 = serial.Serial(micros[0], 115200, 8, 'N', 1, timeout=0.1)
    micro2 = serial.Serial(micros[1], 115200, 8, 'N', 1, timeout=0.1)
    micro1.write('I')
    micro2.write('I')
    m1 = ""
    m2 = ""
    m1 = micro1.readline().strip("\n")
    m2 = micro2.readline().strip("\n")
    while ('M' not in m1):
        m1 = micro1.readline().strip("\n")
    while ('M' not in m2):
        m2 = micro2.readline().strip("\n")
    if (m1 == "M3" and m2 == "M0"):
        rospy.set_param('AER_Driver/control_port', micros[0])
        rospy.set_param('Sensor_Monitor/sensor_port', micros[1])
        print "Case 1"
    if (m1 == "M0" and m2 == "M3"):
        rospy.set_param('AER_Driver/control_port', micros[1])
        rospy.set_param('Sensor_Monitor/sensor_port', micros[0])
        print "Case 2"

    #Clean up
    micro1 = None
    micro2 = None

if __name__ == '__main__':
    identify()
    
