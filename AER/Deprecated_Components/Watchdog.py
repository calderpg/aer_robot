#!/usr/bin/python

import glob
import subprocess

import roslib; roslib.load_manifest('AER')
import rospy

from AER.srv import Respawn

class Watchdog():

    def __init__(self):
        rospy.init_node('Watchdog')
        self.handler = rospy.Service('Respawn', Respawn, self.handler)
        while not rospy.is_shutdown():
            rospy.spin()

    def handler(self, respawn_request):
        node_requesting = respawn_request.node_name
        faulty_port = respawn_request.port_name
        #Get both assigned port numbers:
        control_port = rospy.get_param('AER_Driver/control_port')
        sensor_port = rospy.get_param('Sensor_Monitor/sensor_port')
        #Get a list of actually available ports
        actual_ports = glob.glob('/dev/ttyACM*')
        if (len(actual_ports) > 1):
            #Handle the respawn request
            available_ports = []
            for port_number in actual_ports:
                if port_number != sensor_port:
                    available_ports.append(port_number)
            #Make sure we have a port available
            if (len(available_ports) == 0):
                #If not, send an error code
                return RespawnResponse(2)
            #If we have available ports
            kill_cmd = "rosnode kill /" + node_requesting
            spawn_cmd = "rosrun AER " + node_requesting + ".py&"
            #Kill requesting node
            subprocess.call(kill_cmd, shell=True)
            #Set new parameter
            param_name = 'AER_Driver/control_port'
            rospy.set_param(param_name, available_ports[0])
            #Rerun the node
            subprocess.call(spawn_cmd, shell=True)
        else:
            #Send an error code
            return RespawnResponse(1)

if __name__ == '__main__':
    Watchdog()
