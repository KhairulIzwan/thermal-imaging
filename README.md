# thermal-imaging

Project of wanted to have "heat sensing vision" 

## Required
### Hardware
1. Raspberry PI
2. AMG8833
3. Pi-Camera **or** USB-Camera

### Software
1. Raspbian with ROS Kinetic
2. Raspberry Pi Thermal Camera
	1. https://learn.adafruit.com/adafruit-amg8833-8x8-thermal-camera-sensor/raspberry-pi-thermal-camera

### ROS Package
1. Camera:
	1. cv_camera **or**
		1. git clone https://github.com/OTL/cv_camera.git
	2. usb_cam **or**
		1. sudo apt-get install ros-kinetic-usb-cam
	3. raspicam_node
		1. refer https://github.com/UbiquityRobotics/raspicam_node

2. accessing_raspicam_usbcam:
	1.git clone https://github.com/KhairulIzwan/accessing_raspicam_usbcam.git

3. thermal_imaging:
	1. git clone https://github.com/KhairulIzwan/thermal_imaging.git

## How to use
1. roscore
2. roslaunch accessing_raspicam_usbcam raspicam_robot.launch
3. rosrun thermal_imaging pixels_pub.py
4. rosrun thermal_imaging thermistor_pub.py
5. rosrun thermal_imaging raspicam_preview_rev1.py
