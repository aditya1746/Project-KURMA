#! /usr/bin/env python

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt
from auv_codes2.msg import distance_and_center

img = 0

def image_processing(img):

	detected = 0 # both the flares not detected

	# cases: 
	# a. 2 contours are detected and both are vertical, fine for us
	# b. A single contour is detected for the entire gate, fine for us
	# c. 3 contours are detected for all 3 pipes (one horizontal, two vertical), fine for us

	if(detected == 1):

		msg_dist = distance_and_center()
		msg_distance.centerx = 0
		msg_distance.centery = 0

	pub_dist.publish(msg_dist)

def callback_img(msgs):

    # ============== reading the image from camera_input topic =================
	global img
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msgs, desired_encoding="passthrough")
	
	image_processing(img)

 if __name__ == "__main__":
	
    rospy.init_node("qualification_color_node", anonymous = False)
    q = 1
    
    rospy.Subscriber("Depth", Float32, depth_callback)
    rospy.Subscriber("angle_x", Float64, roll_callback)
	rospy.Subscriber('camera_input', Image, callback_img) 
	
    pub=rospy.Publisher("PWM_VALUE_Middle",String ,queue_size=q)
	pub_dist = rospy.Publisher('gate_pos',distance_and_center,queue_size=10)
    
	# srv = Server(PID_yawConfig, callback_gui)
   
    rospy.spin()
