#!/usr/bin/env python

################################################################################
## {Description}: Accessing and read thermal temp
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
import math

from scipy.interpolate import griddata

# import the necessary ROS packages
from std_msgs.msg import String
from thermal_imaging.msg import pixels

import rospy

class Thermal:
	def __init__(self):

		# Create the Adafruit_AMG88xx object
		self.sensor = Adafruit_AMG88xx()
		self.thermal_received = False

		self.points = [(math.floor(ix / 8), (ix % 8)) for ix in range(0, 64)]
		self.grid_x, self.grid_y = np.mgrid[0:7:32j, 0:7:32j]

		# Connect image topic
		thermal_topic = "/thermal_val"
		self.thermal_pub = rospy.Publisher(thermal_topic, pixels, queue_size=10)

		# Allow up to one second to connection
		rospy.sleep(1)

	def readThermal(self):
		self.getCameraInfo()

		displayPixelWidth = self.image_width / 30
		displayPixelHeight = self.image_height / 30

		thermal_array = np.zeros((self.image_width, self.image_height))

		# Get the pixels array reading
		try:
			pixels_array = self.sensor.readPixels()

			#perdorm interpolation
			bicubic = griddata(self.points, pixels_array, (self.grid_x, self.grid_y), method='cubic')

			# TODO:
			for ix, row in enumerate(bicubic):
				print("*" * 50)
				for jx, pixel in enumerate(row):
					rospy.loginfo("{}, {}, {}, {}, {}".format(pixel, displayPixelHeight * ix, displayPixelWidth * jx, displayPixelHeight, displayPixelWidth))
					thermal_array[displayPixelHeight * ix, displayPixelWidth * jx] = pixel

		except KeyboardInterrupt as e:
			print(e)

		self.thermal_received = True
		self.thermal = thermal_array

	def pubThermal(self):
		self.readThermal()

		if self.thermal_received:
			# Publish thermal array reading
#			self.thermal_pub.publish(self.thermal)
			rospy.loginfo(self.thermal)

		else:
			rospy.logerr("No Thermal reading recieved")

	# Get the width and height of the image
	def getCameraInfo(self):
		self.image_width = rospy.get_param("/raspicam/width") 
		self.image_height = rospy.get_param("/raspicam/height")

if __name__=='__main__':
	# Initializing your ROS Node
	rospy.init_node("thermal", anonymous=False)
 	thermal = Thermal()

	while not rospy.is_shutdown():
		thermal.readThermal()
