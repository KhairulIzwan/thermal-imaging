#!/usr/bin/env python

#Title: Python Subscriber for Tank Navigation
#Author: Khairul Izwan Bin Kamsani - [23-01-2020]
#Description: Tank Navigation Subcriber Nodes (Python)

#remove or add the library/libraries for ROS
import rospy
import sys
import cv2
import imutils
import argparse
import numpy as np

#remove or add the message type
from std_msgs.msg import String

from Adafruit_AMG88xx import Adafruit_AMG88xx

if __name__=='__main__':
	# Initializing your ROS Node
	rospy.init_node("Pixel_Test_Node", anonymous=True)

	# Create the Adafruit_AMG88xx object
	sensor = Adafruit_AMG88xx()

	rate=rospy.Rate(10)
 
	while not rospy.is_shutdown():
		print(sensor.readPixels())
