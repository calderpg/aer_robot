#!/usr/bin/env python

import struct
import roslib; roslib.load_manifest('AER')
import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Kinect_LIDAR():

    def __init__(self):
        rospy.init_node('AER_Kinect_Navigator')

        self.horizscan = None
        self.downscan = None
        downScan_pub = rospy.Publisher('kinect_down_scan', LaserScan)
        horizScan_pub = rospy.Publisher('kinect_horiz_scan', LaserScan)

        rospy.Subscriber("camera/depth/image_rect", Image, self.callback)
        rate = rospy.Rate(rospy.get_param('~hz', 30))
        
        while not rospy.is_shutdown():
            rate.sleep()
            if self.horizscan != None:
                horizScan_pub.publish(self.horizscan)
            if self.downscan != None:
                downScan_pub.publish(self.downscan)

    def callback(self, data):
        """ Receive joystick data, formulate Twist message. """
        horizscan = LaserScan()
        downscan = LaserScan()
        #Set up the 2D array to receive the incoming data
        image_array = [[0.0]*480]*640
        #Convert the 1D array of bytes into the 2D array of floats
        for i in range(640 * 480):
            j = i * 4
            depth_float = struct.unpack("f", data.data[j:j+4])[0]
            x_index = i % 640
            y_index = i / 640
            print float(depth_float)
            image_array[x_index][y_index] = float(depth_float)
        #Create the approximation of a horizontal planar LIDAR scan:
        #averaging the middle 10 pixel-depths across the 640-pixel wide image
        horiz_scan = [0.0]*640
        for i in range(640):
            average_depth = 0.0
            for j in range(235, 245):
                average_depth = average_depth + image_array[i][j]
            average_depth = average_depth / 10.0
            horiz_scan[i] = average_depth
        #Create the approximation of a downwards planar LIDAR scan:
        #averaging the bottom 10 pixel-depths across the 640-pixel wide image
        down_scan = [0.0]*640
        for i in range(640):
            average_depth = 0.0
            for j in range(470, 480):
                average_depth = average_depth + image_array[i][j]
            average_depth = average_depth / 10.0
            down_scan[i] = average_depth
        #Convert these 640-element scans into 64 blocks, each approximately 1 degree of the field of view
        horiz_blocks = [0.0]*64
        for i in range(64):
            block_average = 0.0
            for j in range((i * 10), (i * 10) + 10):
                block_average = block_average + horiz_scan[j]
            horiz_blocks[i] = block_average / 10.0
        down_blocks = [0.0]*64
        for i in range(64):
            block_average = 0.0
            for j in range((i * 10), (i * 10) + 10):
                block_average = block_average + horiz_scan[j]
            down_blocks[i] = block_average / 10.0
        #Convert the 64-element scans into "LaserScan" messages
        horizscan.angle_min = (57.0/2.0) / (Math.Pi * 180.0);
        horizscan.angle_max = -(57.0/2.0) / (Math.Pi * 180.0);
        downscan.angle_min = (57.0/2.0) / (Math.Pi * 180.0);
        downscan.angle_max = -(57.0/2.0) / (Math.Pi * 180.0);
        horizscan.angle_increment = (57.0/64.0) / (Math.Pi * 180.0);
        horizscan.time_increment = 0.0;
        downscan.angle_increment = (57.0/64.0) / (Math.Pi * 180.0);
        downscan.time_increment = 0.0;
        horizscan.range_min = 1.0;
        horizscan.range_max = 5.0;
        downscan.range_min = 1.0;
        downscan.range_max = 5.0;
        horizscan.ranges = horiz_blocks
        downscan.ranges = down_blocks
        #Set up the LaserScan messages to be published
        self.horizscan = horizscan
        self.downscan = downscan

if __name__ == "__main__":
    Kinect_LIDAR()
