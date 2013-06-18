#!/bin/bash
echo "Starting Kinect Navigator"
roslaunch openni_launch openni.launch&
rosrun AER Kinect_Depth_Navigator
