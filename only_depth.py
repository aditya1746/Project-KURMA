#! /usr/bin/env python
from numpy import float32
import rospy
from dynamic_reconfigure.server import Server
#from auv_codes2.cfg import thrusterConfig
from auv_codes2.cfg import PID_yawConfig
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import String


#    FL BL FR BR ML MR
# F  C  C  C  C 
# B  A  A  A  A
# L  A  C  C  A
# R  C  A  A  C
# D  -  -  -  -  C  C
# U  -  -  -  -  A  A

prev_err_d, int_err_d = 0,0         # d -> depth
err_d_thresh = 0.007                # meters
prev_err_r, int_err_r = 0,0         # r -> roll
err_r_thresh = 10                   # degrees

depth = 0
desiredDepth = 0.7                  # meters
roll = 0

KP_d, KD_d, KI_d = 1,1,1

MAXpwm, MINpwm = 400, -400
pwmBase = 1470
pwm_mr, pwm_ml = 100 , 100  # ================== 100 basically represent base velocities (1470)

# x-axis ^ 
# y-axis >

def depth_pid_pressure():
	global prev_err_d, int_err_d
	global pwm_mr, pwm_ml
	# pwm_mr, pwm_ml = 100, 100  # ================== 100 basically represent base velocities
	
	err_d = desiredDepth - depth # +err => or niche jana he
	diff_err_d = err_d - prev_err_d
	int_err_d += err_d 
    prev_err_d = err_d
	
	correction_d = KP_d*abs(err_d) + KD_d*diff_err_d
	if(int_err_d*err_d > 0) :
		correction_d = correction_d + KI_d*int_err_d 
	else:
	 	correction_d =correction_d - KI_d*int_err_d 
	correction_d = max(correction_d,0)
	
	if(abs(err_d) > err_d_thresh):
		pwm_mr, pwm_ml = 100, 100

	if(err_d > err_d_thresh):
		pwm_mr += correction_d # increasing the force of both thrusters
        pwm_ml += correction_d
        pwm_mr = min(pwm_mr, MAXpwm)
        pwm_ml = min(pwm_ml, MAXpwm)
	
	elif(err_y < -err_y_thresh):
		pwm_ml -= correction_d
        pwm_mr -= correction_d
        pwm_mr = max(pwm_mr, MINpwm)
        pwm_ml = max(pwm_ml, MINpwm)
	
	elif(abs(err_r) > err_r_thresh):  # what if, auv is rolled towards one side, which is really rare, thus keeping it simple
		if(err_r>0): # \ this back slash represents the line joining middle right and left thruster when seen from back 
			pwm_ml += err_r
            pwm_ml = min(pwm_ml, MAXpwm)
		else:
            pwm_mr += err_r
            pwm_mr = min(pwm_mr, MAXpwm)
			
	pwm_mr, pwm_ml = pwmBase + int(pwm_mr), pwmBase - 1*int(pwm_ml)

	pwm_msg = str(pwm_mr) + ' ' + str(pwm_ml) + ' '
	pub.publish(pwm_msg)
	rospy.loginfo(depth)

def stopMiddle():
	pwm_mr, pwm_ml = pwmBase, pwmBase
	pwm_msg = str(pwm_mr) + ' ' + str(pwm_ml) + ' '
	pub.publish(pwm_msg)

def startMiddle():
	pwm_mr, pwm_ml = 100, 100
	pwm_mr, pwm_ml = pwmBase + int(pwm_mr), pwmBase - 1*int(pwm_ml)
	pwm_msg = str(pwm_mr) + ' ' + str(pwm_ml) + ' '
	pub.publish(pwm_msg)
	

def callback_gui(config, level):
    rospy.loginfo("""Reconfigure Request: {KP_depth}, {KI_depth}, {KD_depth}""".format(**config))
    global KP_d, KI_d, KD_d
    KP_d, KI_d, KD_d = config.KP_depth / 100, config.KI_depth / 100, config.KD_depth / 100 
    return config

def depth_callback(msg):
	global depth
	depth = msg.data
    depth_pid_pressure()

def depthCBinGUI(msg):
	global depth
	depth = msg
    depth_pid_pressure()

def roll_callback(msg):
	global roll
	roll = msg.data

if __name__ == "__main__":
	
	try:
		rospy.init_node("only_depth_pid", anonymous = False)
		q = 1
		
		rospy.Subscriber("Depth", Float32, depth_callback)
		rospy.Subscriber("angle_x", Float64, roll_callback)
		
		pub=rospy.Publisher("PWM_VALUE_Middle",String ,queue_size=q)
		srv = Server(PID_yawConfig, callback_gui)
	
		rospy.spin()
	except KeyboardInterrupt:
		stopMiddle()
