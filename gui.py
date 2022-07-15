#! /usr/bin/env python

# from sympy import true
import rospy
from dynamic_reconfigure.server import Server
from auv_codes2.cfg import thrusterConfig
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from only_straight import *
from only_depth import *

mode = 0

def cb_verify(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def depth_cb(msg):

    if(mode == 3):
        depthCBinGUI(msg.data)

def yaw_cb(msg):

    if(mode == 4):
        yawCBinGUI(msg.data)

def cb_conf(config, level):
    global mode

    rospy.loginfo("""Reconfigure Request: {servoFrontRight}, {servoFrontLeft},{servoMiddleRight}, {servoMiddleLeft}, {servoBackLeft}, {servoBackRight},{mode_thrusters}""".format(**config))

    mode   = config.mode_thrusters
    pwm_fr, pwm_br, pwm_fl, pwm_bl, pwm_mr, pwm_ml =0, 0, 0, 0, 0, 0

    if (mode==0): #stop all thrusters
        # print('mode is ', mode)
        # pwm_fr, pwm_br, pwm_fl, pwm_bl, pwm_mr, pwm_ml =0, 0, 0, 0, 0, 0
        stopCar()
        stopMiddle()

    elif  (mode==1): #all thrusters value to 1600 to check every thruster is working
        # print('mode is ', mode)   
        # pwm_fr, pwm_br, pwm_fl, pwm_bl, pwm_mr, pwm_ml =80, 80, 80, 80, 80, 80
        startCar()
        startMiddle()

    elif  (mode==2):   # value receieved from reconfigure
        # print('mode is ', mode)
        pwm_fr = config.servoFrontRight
        pwm_fl = config.servoFrontLeft
        pwm_mr = config.servoMiddleRight
        pwm_ml = config.servoMiddleLeft
        pwm_br = config.servoBackRight
        pwm_bl = config.servoBackLeft

    pwm_fr, pwm_br, pwm_fl, pwm_bl , pwm_mr, pwm_ml = pwmBase - int(pwm_fr), pwmBase +int(pwm_br),pwmBase + int(pwm_fl), pwmBase +int(pwm_bl), pwmBase + int(pwm_mr), pwmBase - int(pwm_ml)
    
    pwm_value= str(pwm_fr) +' '+ str(pwm_fl) +' '+str(pwm_mr)+ ' '+ str(pwm_ml)+ ' '+ str(pwm_br)+' '+ str(pwm_bl)
    pub.publish(pwm_value) 

    return config

if __name__ == "__main__":
    rospy.init_node("thruster_value", anonymous = False)
    q = 1
    pub=rospy.Publisher("PWM_VALUE",String ,queue_size=q)

    rospy.Subscriber("Depth", Float32, depth_cb)
    rospy.Subscriber("angle_z", Float64, yaw_cb)
    rospy.Subscriber("Verify", String, cb_verify)
    srv = Server(thrusterConfig, cb_conf)
    rospy.spin()
