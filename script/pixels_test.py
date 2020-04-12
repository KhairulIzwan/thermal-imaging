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

#class PixelTest:

#	def __init__(self):
#		# Initializing your ROS Node
#		rospy.init_node("Pixel_Test_Node", anonymous=True)

#		rospy.on_shutdown(self.shutdown)

#		# Create the Adafruit_AMG88xx object
#		self.sensor = Adafruit_AMG88xx()

##		# Create the Subsciber (image_raw)
##		self.sub = rospy.Subscriber("/raspicam_node_robot/image/compressed", 
##				CompressedImage, self.callback, queue_size=1)

##		# Create the Publisher (roi)		
##		self.pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=10)

#	#
#	def test_pixel(self):
#		print(self.sensor.readPixels())

#	# Get the width and height of the image
#	def getCameraInfo(self):
#		self.image_width = rospy.get_param("/raspicam_node_robot/width") 
#		self.image_height = rospy.get_param("/raspicam_node_robot/height") 
#		rospy.set_param("/raspicam_node_robot/brightness", 1000)
#	
#	def callback(self,data):
#		# Convert the raw image to OpenCV format
#		self.cvtImage(data)

#		# Get the width and height of the image
#		self.getCameraInfo()

#		# Detect face
#		self.track()

#		# Publish ROI
#		self.publishROI()

#		# Refresh the image on the screen
#		self.displayImg()

#	# Convert the raw image to OpenCV format
#	def cvtImage(self, data):
#		try:
#			# Convert the raw image to OpenCV format """
#			# self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

#			# direct conversion to CV2 ####
#			self.cv_image = np.fromstring(data.data, np.uint8)
#			self.cv_image = cv2.imdecode(self.cv_image, cv2.IMREAD_COLOR)

#			# OTIONAL -- image-rotate """
#			self.cv_image = imutils.rotate(self.cv_image, angle=-90)

#			# Clone the original image for displaying purpose later
#			self.frameClone = self.cv_image.copy()

#		except CvBridgeError as e:
#			print(e)

#	def track(self):
#		# Create an empty arrays for save rects value later
#		self.rects = []
#		
#		# Detect all faces in the input frame
#		faceRects = self.faceCascade.detectMultiScale(self.cv_image,
#			scaleFactor = 1.1, minNeighbors = 5, minSize = (30, 30),
#			flags = cv2.CASCADE_SCALE_IMAGE)

#		# Loop over the face bounding boxes
#		for (fX, fY, fW, fH) in faceRects:
#			# Extract the face ROI and update the list of bounding boxes
#			faceROI = self.cv_image[fY:fY + fH, fX:fX + fW]
#			self.rects.append((fX, fY, fX + fW, fY + fH))

#	def publishROI(self):
#		# loop over the face bounding boxes and draw them
#		for rect in self.rects:
#			cv2.rectangle(self.frameClone, (rect[0], rect[1]), (rect[2], rect[3]), (0, 255, 0), 2)

#			roi=RegionOfInterest()
#			roi.x_offset=rect[0]
#			roi.y_offset=rect[1]
#			roi.width=rect[2]
#			roi.height=rect[3]

#			self.pub.publish(roi)

#	# Refresh the image on the screen
#	def displayImg(self):
#		cv2.imshow("Face Detector", self.frameClone)
#		cv2.waitKey(1)

#	def shutdown(self):
#		try:
#			rospy.loginfo("[INFO] Pixel Test Node [OFFLINE]")
#		finally:
#			cv2.destroyAllWindows()

#def main(args):
#	tfd = PixelTest()
#	try:
#		tfd.test_pixel()
#		rospy.spin()
#	except ROSInterruptException:
#		rospy.loginfo("[INFO] Pixel Test Node [OFFLINE]")

#	cv2.destroyAllWindows()

#if __name__ == "__main__":
#	rospy.loginfo("[INFO] Pixel Test Node [ONLINE]")
#	main(sys.argv)

if __name__=='__main__':
	# Initializing your ROS Node
	rospy.init_node("Pixel_Test_Node", anonymous=True)

	# Create the Adafruit_AMG88xx object
	sensor = Adafruit_AMG88xx()

	rate=rospy.Rate(10)
 
	while not rospy.is_shutdown():
		print(sensor.readPixels())
