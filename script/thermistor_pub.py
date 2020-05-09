#!/usr/bin/env python

################################################################################
## {Description}: Accessing and read thermal temperature (surrounding)
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

class Thermistor:
	def __init__(self):

		# Create the Adafruit_AMG88xx object
		self.sensor = Adafruit_AMG88xx()
		self.temp_received = False

		# Connect image topic
		temp_topic = "/thermistor_temp"
		self.temp_pub = rospy.Publisher(temp_topic, Float64, queue_size=10)

		# Allow up to one second to connection
		rospy.sleep(1)

	def readTemp(self):

		# Get the thermistor temp reading
		try:
			thermal_temp = self.sensor.readThermistor()
		except KeyboardInterrupt as e:
			print(e)

		self.temp_received = True
		self.temp = thermal_temp

	def pubTemp(self):
		self.readTemp()

		if self.temp_received:
			# Publish Thermistor Temp reading
			self.temp_pub.publish(self.temp)

		else:
			rospy.logerr("No Thermistor Temp recieved")

if __name__=='__main__':
	# Initializing your ROS Node
	rospy.init_node("thermistor", anonymous=False)
	thermal = Thermistor()
 
	while not rospy.is_shutdown():
		thermal.pubTemp()
