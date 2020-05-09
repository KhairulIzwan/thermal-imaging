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
from thermal_imaging.msg import pixels

import rospy

class Pixels:
	def __init__(self):

		# Create the Adafruit_AMG88xx object
		self.sensor = Adafruit_AMG88xx()
		self.pixels_received = False

		# Connect image topic
		pixels_topic = "/pixels"
		self.pixels_pub = rospy.Publisher(pixels_topic, pixels, queue_size=10)

		# Allow up to one second to connection
		rospy.sleep(1)

	def readPixels(self):

		# Get the pixels array reading
		try:
			pixels_array = self.sensor.readPixels()
		except KeyboardInterrupt as e:
			print(e)

		self.pixels_received = True
		self.pixels = pixels_array

	def pubPixels(self):
		self.readPixels()

		if self.pixels_received:
			# Publish pixels array reading
			self.pixels_pub.publish(self.pixels)

		else:
			rospy.logerr("No Pixels reading recieved")

if __name__=='__main__':
	# Initializing your ROS Node
	rospy.init_node("pixels", anonymous=False)
 	pixels = Pixels()

	while not rospy.is_shutdown():
		pixels.pubPixels()
