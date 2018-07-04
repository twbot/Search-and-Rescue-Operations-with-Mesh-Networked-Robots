#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import time
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetMode

# some variables to define the behavior of the robot (hardcoded, modify at your convenience)
throttle_channel=2
steer_channel=0

def autopilot_abstraction(speed='SLOW',direction='STRAIGHT', exec_time=1):
 pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=10)
 r = rospy.Rate(10) #10hz
 msg = OverrideRCIn()
 start = time.time()
 flag=True #time flag

 # Abstract the speed of the Rover
 if speed =='SLOW':
  msg.channels[throttle_channel]=1558
 elif speed =='NORMAL':
  msg.channels[throttle_channel]=1565
 elif speed == 'FAST':
  msg.channels[throttle_channel]=1570

 # Abstract the direction of the Rover
 if direction =='STRAIGHT':
  msg.channels[steer_channel]=1500
 elif direction =='LEFT':
  msg.channels[steer_channel]=1200
 elif direction == 'RIGHT':
  msg.channels[steer_channel]= 1800

 while not rospy.is_shutdown() and flag:
  sample_time=time.time()
  if ((sample_time - start) > exec_time):
   flag=False
  rospy.loginfo(msg)
  pub.publish(msg)
  r.sleep()

if __name__ == '__main__':
 rospy.init_node('tryrover_node', anonymous=True)
 rospy.wait_for_service('/mavros/set_mode')
 print('Waiting for set_mode response...\n')
 change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
 answer = change_mode(custom_mode='manual')
 print('Mode changed to manual?')
 print (answer)
 if 'True' in str(answer):
  try:
   autopilot_abstraction('SLOW','RIGHT', 2)
   autopilot_abstraction('SLOW','STRAIGHT', 2)
   autopilot_abstraction('SLOW','LEFT', 2)

  except rospy.ROSInterruptException: pass