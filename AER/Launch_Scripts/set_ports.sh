#!/bin/bash
echo "Setting hardware control variables to the raw device IDs"
rosparam set "AER_Driver/control_port" "/dev/serial/by-id/usb-mbed_Microcontroller_101000000000000000000002F7F09405-if01"
rosparam set "Sensor_Monitor/sensor_port" "/dev/serial/by-id/usb-mbed_Microcontroller_104000000000000000000002F7F0E808-if01"
