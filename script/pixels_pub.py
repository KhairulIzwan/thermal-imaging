#!/usr/bin/env python

################################################################################
## {Description}: Accessing and read pixels temp
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
from __future__ import division

from Adafruit_AMG88xx import Adafruit_AMG88xx

import sys
import cv2
import time
import numpy as np
import imutils

# import the necessary ROS packages
from std_msgs.msg import String
from std_msgs.msg import Float64

import rospy

if __name__=='__main__':
	# Initializing your ROS Node
	rospy.init_node("Pixel_Test_Node", anonymous=True)

	# Create the Adafruit_AMG88xx object
	sensor = Adafruit_AMG88xx()

	rate=rospy.Rate(10)
 
	while not rospy.is_shutdown():
		print(sensor.readPixels())
