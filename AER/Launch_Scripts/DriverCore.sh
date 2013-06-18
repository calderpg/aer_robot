#!/bin/bash
echo "Starting Core Driver!"
rosnode kill -a
roscore&
echo "Running ROS nodes killed and roscore checked"
./set_ports.sh
echo "Port settings configured"
echo "Starting platform driver"
rosrun AER Power_Monitor.py&
rosrun AER Sensor_Monitor.py&
rosrun AER AER_Driver.py
