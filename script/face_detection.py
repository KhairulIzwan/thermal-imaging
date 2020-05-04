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
import rospkg
import os

#remove or add the message type
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import RegionOfInterest

class FaceDetector:

	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node("Face_Detector_Node", anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Create the cv_bridge object
		self.bridge = CvBridge()

		# Path to input Haar cascade for face detection
		self.rospack = rospkg.RosPack()
		self.p = os.path.sep.join([self.rospack.get_path('thermal-imaging')])
		self.outputDir = os.path.join(self.p, "library")

		self.haar_filename = self.outputDir + "/haarcascade_frontalface_default.xml"
		self.faceCascade = cv2.CascadeClassifier(self.haar_filename)

		# Create the Subsciber (image_raw)
		self.sub = rospy.Subscriber("/raspicam_node_robot/image/compressed", CompressedImage)

		# Create the Publisher (roi)		
		self.pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=10)

		self.dispImage()

	# Get the image raw
	def getImage(self):
		# Wait for the topic
		self.image = rospy.wait_for_message("/raspicam_node_robot/image/compressed", CompressedImage)

	# Get the width and height of the image
	def getCameraInfo(self):
		self.image_width = rospy.get_param("/raspicam_node_robot/width") 
		self.image_height = rospy.get_param("/raspicam_node_robot/height")

	# Convert the raw image to OpenCV format
	def cvtImage(self):
		# Get the scan-ed data
		self.getImage()

		# direct conversion to CV2 ####
		self.cv_image = np.fromstring(self.image.data, np.uint8)
		self.cv_image = cv2.imdecode(self.cv_image, cv2.IMREAD_COLOR)

		# OPTIONAL -- image-rotate """
		self.cv_image = imutils.rotate(self.cv_image, angle=-90)
		self.cv_image = cv2.flip(self.cv_image, 1)

		# Clone the original image for displaying purpose later
		self.frameClone = self.cv_image.copy()

	def track(self):
		# Create an empty arrays for save rects value later
		self.rects = []
		
		# Detect all faces in the input frame
		faceRects = self.faceCascade.detectMultiScale(self.cv_image, scaleFactor = 1.1, minNeighbors = 5, minSize = (30, 30), flags = cv2.CASCADE_SCALE_IMAGE)

		# Loop over the face bounding boxes
		for (fX, fY, fW, fH) in faceRects:
			# Extract the face ROI and update the list of bounding boxes
			faceROI = self.cv_image[fY:fY + fH, fX:fX + fW]
			self.rects.append((fX, fY, fX + fW, fY + fH))

	def publishROI(self):
		# loop over the face bounding boxes and draw them
		for rect in self.rects:
			cv2.rectangle(self.frameClone, (rect[0], rect[1]), (rect[2], rect[3]), (0, 255, 0), 2)

			roi=RegionOfInterest()
			roi.x_offset=rect[0]
			roi.y_offset=rect[1]
			roi.width=rect[2]
			roi.height=rect[3]

			self.pub.publish(roi)

	# Refresh the image on the screen
	def dispImage(self):
		while not rospy.is_shutdown():
			try:
				# Get the scan-ed data
				self.cvtImage()
				self.track()
				self.publishROI()

				cv2.imshow("Face Detector", self.frameClone)
				cv2.waitKey(1)

			except CvBridgeError as e:
				print(e)

	def shutdown(self):
		try:
			rospy.loginfo("[INFO] Face Detector Node [OFFLINE]")
		finally:
			cv2.destroyAllWindows()

def main(args):
	tfd = FaceDetector()
	try:
		rospy.spin()
	except ROSInterruptException:
		rospy.loginfo("[INFO] Face Detector Node [OFFLINE]")

	cv2.destroyAllWindows()

if __name__ == "__main__":
	rospy.loginfo("[INFO] Face Detector Node [ONLINE]")
	main(sys.argv)
