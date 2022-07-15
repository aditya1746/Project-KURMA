#! /usr/bin/env python
from numpy import float32
import rospy
from dynamic_reconfigure.server import Server
#from auv_codes2.cfg import thrusterConfig
from auv_codes2.cfg import PID_yawConfig
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String


#    FL BL FR BR ML MR
# F  C  C  C  C 
# B  A  A  A  A
# L  A  C  C  A
# R  C  A  A  C
# D  -  -  -  -  C  C
# U  -  -  -  -  A  A

pwmBase = 1470
yaw_desired, cnt = 0, 0
yaw = 0
acc_y = 0
prev_err_y, int_err_y, prev_err_a, int_err_a = 0,0,0,0

KP_y, KD_y, KI_y = 1,1,1
KP_a, KD_a, KI_a = 1,1,1

err_y_thresh = 4 #degrees
err_a_thresh = 0
MAXpwm, MINpwm = 400, -400

pwm_fr, pwm_fl, pwm_br, pwm_bl = 100, 100, 100, 100  # ================== 100 basically represent base velocities

# x-axis ^ 
# y-axis >
# err_y is positive => robot is turned towards right, when we are looking from back
# err_a is positive => robot is moving in y axis

def straightLine_pid_imu():
	global prev_err_y, int_err_y, prev_err_a, int_err_a
	global pwm_fr, pwm_fl, pwm_br, pwm_bl
	# pwm_fr, pwm_br, pwm_fl, pwm_bl = 100, 100, 100, 100 # ================== 100 basically represent base velocities
	
	err_y = yaw - yaw_desired
	diff_err_y = err_y - prev_err_y
	int_err_y += err_y 
	prev_err_y = err_y
	
	correction_y = KP_y*abs(err_y) + KD_y*diff_err_y
	if(int_err_y*err_y > 0) :
		correction_y = correction_y + KI_y*int_err_y 
	else:
	 	correction_y =correction_y - KI_y*int_err_y 
	correction_y = max(correction_y,0)
		
	err_a = acc_y
	diff_err_a = err_a - prev_err_a
	int_err_a += err_a 
	prev_err_a = err_a
	
	correction_a = KP_a*abs(err_a) + KD_a*diff_err_a
	if(int_err_a*err_a > 0) :
		correction_a = correction_a + KI_a*int_err_a 
	else:
	 	correction_a =correction_a - KI_a*int_err_a 
	correction_a = max(correction_a,0)
	
	if(abs(err_y) > err_y_thresh):
		pwm_fr, pwm_br, pwm_fl, pwm_bl = 100, 100, 100, 100

	if(err_y > err_y_thresh):
		pwm_fr += correction_y # increasing the force of front right
		# pwm_bl -= correction_y # decreasing the force of back left, indeed we could change any of the 4ci, so for the simplicity lets change only one
		pwm_fr = min(pwm_fr, MAXpwm)
        # pwm_bl = max(pwm_bl, 0)
	
	elif(err_y < -err_y_thresh):
		pwm_fl += correction_y
		# pwm_br -= correction_y
		pwm_fl = min(pwm_fl, MAXpwm)
        # pwm_br = max(pwm_br, 0)
	
	elif(abs(err_a) > err_a_thresh):  #what if, auv is facing straight but moving left or right or diagnonally. 
		if(err_a>0): # auv is moving towards right
			pwm_fr += correction_a # increase the force of front right and back left
			pwm_bl += correction_a # and yes here we need to change both or all 4, so that no turning force is created
			pwm_fr = min(pwm_fr, MAXpwm)
			pwm_bl = min(pwm_bl, MAXpwm) 
		else:
			pwm_fl += correction_a
			pwm_br += correction_a
			pwm_fl = min(pwm_fl, MAXpwm)
			pwm_br = min(pwm_br, MAXpwm)

	pwm_fr, pwm_br, pwm_fl, pwm_bl = pwmBase - 1*int(pwm_fr), pwmBase +int(pwm_br),pwmBase + int(pwm_fl), pwmBase +int(pwm_bl)
	pwm_msg = str(pwm_fr) + ' ' + str(pwm_fl) + ' ' + str(pwm_br) + ' ' + str(pwm_bl) + ' '		
	pub.publish(pwm_msg)
	rospy.loginfo(yaw)
	

# 	return err_y, correction_y, err_a, correction_a

# def moveForward:
# 	err_y, correction_y, err_a, correction_a = straightLine_pid_imu()

def stopCar():
	global pwm_fr, pwm_br, pwm_fl, pwm_bl 
	pwm_fr, pwm_br, pwm_fl, pwm_bl = pwmBase, pwmBase, pwmBase, pwmBase
	pwm_msg = str(pwm_fr) + ' ' + str(pwm_fl) + ' ' + str(pwm_br) + ' ' + str(pwm_bl) + ' '		
	pub.publish(pwm_msg)
	
def accy_callback(msg):
	global acc_y
	acc_y = msg.data

def accx_callback(msg):
	global acc_x
	acc_x = msg.data

def yaw_callback(msg):
	global yaw, yaw_desired, cnt
	yaw = msg.data
	if(cnt==0):
		yaw_desired=yaw
		cnt = 1
	
	straightLine_pid_imu()
	
def callback_gui(config, level):
    rospy.loginfo("""Reconfigure Request: {KP_yaw}, {KI_yaw}, {KD_yaw}""".format(**config))
    global KP_y, KI_y, KD_y
    KP_y, KI_y, KD_y = config.KP_yaw / 100, config.KI_yaw / 100, config.KD_yaw / 100 
    return config

if __name__ == "__main__":
	
	try:
		rospy.init_node("only_straight_pid", anonymous = False)
		q = 1
		
		rospy.Subscriber("angle_z", Float64, yaw_callback)
		rospy.Subscriber("accx", Float64, accx_callback)
		rospy.Subscriber("accy", Float64, accy_callback)
		
		pub=rospy.Publisher("PWM_VALUE_car",String ,queue_size=q)
		srv = Server(PID_yawConfig, callback_gui)
	
		rospy.spin()

	except KeyboardInterrupt:
		stopCar()
