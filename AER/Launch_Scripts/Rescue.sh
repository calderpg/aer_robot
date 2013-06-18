#!/bin/bash
echo "Starting Rescue Driver!"
rosnode kill -a
roscore&
echo "Running ROS nodes killed and roscore checked"
rosparam set joy_node/dev "/dev/input/js0"
./set_ports.sh
echo "Port settings configured"
rosrun joy joy_node&
rosrun AER AER_Teleop.py&
echo "Starting platform driver"
rosrun AER AER_Driver.py
