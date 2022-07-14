#! /usr/bin/env python

# from sympy import true
import rospy

from dynamic_reconfigure.server import Server
from auv_codes2.cfg import thrusterConfig
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import String


pwmBase = 1470

def callback_verify(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)



def callback_conf(config, level):
    rospy.loginfo("""Reconfigure Request: {servoFrontRight}, {servoFrontLeft},{servoMiddleRight}, {servoMiddleLeft}, {servoBackLeft}, {servoBackRight},{mode_thrusters}""".format(**config))
   
    mode   = config.mode_thrusters
    pwm_fr, pwm_br, pwm_fl, pwm_bl, pwm_mr, pwm_ml =0, 0, 0, 0, 0, 0
    if (mode==0):
            #stop all thrusters
        print('mode is ', mode)
        pwm_fr, pwm_br, pwm_fl, pwm_bl, pwm_mr, pwm_ml =0, 0, 0, 0, 0, 0

    elif  (mode==1):
               #all thrusters value to 1600 to check every thruster is working
        print('mode is ', mode)   
        pwm_fr, pwm_br, pwm_fl, pwm_bl, pwm_mr, pwm_ml =80, 80, 80, 80, 80, 80

    elif  (mode==2):  
        
              # value receieved from reconfigure
        print('mode is ', mode)
        pwm_fr = config.servoFrontRight
        pwm_fl = config.servoFrontLeft
        pwm_mr = config.servoMiddleRight
        pwm_ml = config.servoMiddleLeft
        pwm_br = config.servoBackRight
        pwm_bl = config.servoBackLeft

    elif  (mode==3): 
        print('mode is ', mode)

    elif  (mode==4):         # value from imu sensor 

        print('mode is ', mode)
     
        
    elif  (mode==5):        # value from camera 
        print('mode is ', mode)

    pwm_fr, pwm_br, pwm_fl, pwm_bl , pwm_mr, pwm_ml = pwmBase - int(pwm_fr), pwmBase +int(pwm_br),pwmBase + int(pwm_fl), pwmBase +int(pwm_bl), pwmBase + int(pwm_mr), pwmBase - int(pwm_ml)
    
    pwm_value= str(pwm_fr) +' '+ str(pwm_fl) +' '+str(pwm_mr)+ ' '+ str(pwm_ml)+ ' '+ str(pwm_br)+' '+ str(pwm_bl)
    pub.publish(pwm_value) 

    print("Out of if else")
    
           
    # pub7.publish(stop)


    return config

if __name__ == "__main__":
    rospy.init_node("thruster_value", anonymous = False)
    q = 1
    pub=rospy.Publisher("PWM_VALUE",String ,queue_size=q)
    rospy.Subscriber("Verify", String, callback_verify)
    srv = Server(thrusterConfig, callback_conf)
    rospy.spin()
