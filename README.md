# thermal-imaging

Project of wanted to have "heat sensing vision" 

## Required
### Hardware
1. Raspberry PI
2. AMG8833
3. Pi-Camera or USB-Camera

### Software
1. Raspbian with ROS Kinetic
2. Raspberry Pi Thermal Camera
	1. https://learn.adafruit.com/adafruit-amg8833-8x8-thermal-camera-sensor/raspberry-pi-thermal-camera

### ROS Package
1. Camera:
	1. cv_camera
		1. git clone https://github.com/OTL/cv_camera.git
	2. usb_cam
		1. sudo apt-get install ros-kinetic-usb-cam
	3. raspicam_node
		1. refer https://github.com/UbiquityRobotics/raspicam_node


