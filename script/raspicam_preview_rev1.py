#!/usr/bin/env python

################################################################################
## {Description}: Preview an Image from Raspberry Pi Camera (raspicam)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
from __future__ import division

import sys
import cv2
import time
import numpy as np
import imutils
import os
import dlib
from imutils import face_utils
import math

from scipy.interpolate import griddata

# import the necessary ROS packages
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from std_msgs.msg import Float64

from thermal_imaging.msg import pixels

import rospy
import rospkg

class RaspicamPreview:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False

		self.rospack = rospkg.RosPack()
		self.p = os.path.sep.join([self.rospack.get_path('common_face_application')])
		self.libraryDir = os.path.join(self.p, "library")

		self.dlib_filename = self.libraryDir + "/shape_predictor_68_face_landmarks.dat"

		# initialize dlib's face detector (HOG-based) and then create 
		# the facial landmark predictor
		rospy.loginfo("Loading facial landmark predictor...")
		self.detector = dlib.get_frontal_face_detector()
		self.predictor = dlib.shape_predictor(self.dlib_filename)

		self.points = [(math.floor(ix / 8), (ix % 8)) for ix in range(0, 64)]
		self.grid_x, self.grid_y = np.mgrid[0:7:32j, 0:7:32j]

		# Connect image topic
		img_topic = "/raspicam/image/compressed"
		self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.callback)

		# Connect to Thermistor Temp topic
		self.temp_topic = "/thermistor_temp"
		self.temp_sub = rospy.Subscriber(self.temp_topic, Float64)

		# Connect to Thermistor Temp topic
		self.pixels_topic = "/pixels_val"
		self.pixels_sub = rospy.Subscriber(self.pixels_topic, pixels)

		# Allow up to one second to connection
		rospy.sleep(1)

	def callback(self, data):

		# Convert image to OpenCV format
		try:
			cv_image = np.fromstring(data.data, np.uint8)
			cv_image = cv2.imdecode(cv_image, cv2.IMREAD_COLOR)

			# OPTIONAL -- image-rotate """
			cv_image = imutils.rotate(cv_image, angle=-90)

		except CvBridgeError as e:
			print(e)

		self.image_received = True
		self.image = cv_image

	# show the output image and the final shape count
	def preview(self):

		#
		self.readTemp()
		self.readPixels()
		self.getCameraInfo()

		if self.image_received:
			#
			self.detectFacialLandmark()

			# Overlay some text onto the image display
#			timestr = time.strftime("%Y%m%d-%H%M%S")
#			cv2.putText(self.image, timestr, (10, self.image_height-20), 
#				1, 1, (255, 255, 255), 1, cv2.LINE_AA, False)
			cv2.putText(self.image, "{0:0.2f}".format(self.room_temp), 
				(10, 20), 1, 1, (255, 255, 255), 1, cv2.LINE_AA, 
				False)
#			cv2.putText(self.image, "{}, {}".format(self.image_width, self.image_height), 
#				(self.image_width-80, 20), 1, 1, (255, 255, 255), 1, cv2.LINE_AA, 
#				False)

			# show the output frame
			cv2.imshow("Frame", self.image)
			cv2.waitKey(1)

		else:
			rospy.logerr("No images recieved")

	def readTemp(self):
		# Wait for the topic
		self.temp = rospy.wait_for_message(self.temp_topic, Float64)
		self.room_temp = self.temp.data

	def readPixels(self):
		self.getCameraInfo()

		displayPixelWidth = self.image_width / 30
		displayPixelHeight = self.image_height / 30

		thermal_array = np.zeros((self.image_width + 100, self.image_height + 100))

		# Wait for the topic
		self.pixels_temp = rospy.wait_for_message(self.pixels_topic, pixels)

		#perdorm interpolation
		bicubic = griddata(self.points, self.pixels_temp.data, (self.grid_x, self.grid_y), method='cubic')

		for ix, row in enumerate(bicubic):
			for jx, pixel in enumerate(row):
				#rospy.loginfo("{}, {}, {}, {}, {}".format(pixel, displayPixelHeight * ix, displayPixelWidth * jx, displayPixelHeight, displayPixelWidth))
				thermal_array[int(displayPixelHeight * ix), int(displayPixelWidth * jx)] = pixel

	# Get the width and height of the image
	def getCameraInfo(self):
		self.image_width = rospy.get_param("/raspicam/width") 
		self.image_height = rospy.get_param("/raspicam/height")

	def detectFacialLandmark(self):
		gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

		# detect faces in the grayscale frame
		rects = self.detector(gray, 0)

		# loop over the face detections
		for (i, rect) in enumerate(rects):
			# determine the facial landmarks for the face region, then
			# convert the facial landmark (x, y)-coordinates to a NumPy
			# array
			shape = self.predictor(gray, rect)
			shape = face_utils.shape_to_np(shape)

			# convert dlib's rectangle to a OpenCV-style bounding box
			# [i.e., (x, y, w, h)], then draw the face bounding box
			(self.x, self.y, self.w, self.h) = face_utils.rect_to_bb(rect)
			cv2.rectangle(self.image, (self.x, self.y), 
				(self.x + self.w, self.y + self.h), (0, 255, 0), 2)

			# show the face number
			cv2.putText(self.image, "Face #{}".format(i + 1), (self.x - 10, self.y - 10),
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

			cv2.circle(self.image, (self.x + (self.w // 2), self.y + (self.h // 2)), 3, (0, 0, 255), -1)

			cv2.putText(self.image, "{}, {}".format(self.x + (self.w // 2), self.y + (self.h // 2)), 
				(self.image_width-80, 20), 1, 1, (255, 255, 255), 1, cv2.LINE_AA, 
				False)

#			# loop over the (x, y)-coordinates for the facial landmarks
#			# and draw them on the image
#			for (x, y) in shape:
#				cv2.circle(self.image, (x, y), 1, (0, 0, 255), -1)

if __name__ == '__main__':

	# Initialize
	rospy.init_node('raspicam_preview', anonymous=False)
	camera = RaspicamPreview()

	# Camera preview
	while not rospy.is_shutdown():
		camera.preview()
