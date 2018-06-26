#!/usr/bin/env python

import rospy

import cv2
import numpy as np
import math

from sensor_msgs.msg import Image
from mavros_msgs.msg import OverrideRCIn

from digi.xbee.devices import ZigBeeDevice

pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

class RoverState():
  def __init__(self):
        self.start_time = None # To record the start time of navigation
        self.total_time = None # To record total duration of naviagation
        self.start_pos = None
        self.last_pos = None
        self.forward_time = 0 # To record total time going forward
        self.stuck_time = 0 # To record amount of time stuck
        self.max_stuck_time = 4 # Max time before determined stuck
        self.steer_time = 0 # To record amount of time steering same angle
        self.max_steer_time = 6 # Max time before determined circle steering
        self.duration_steer = 0
        self.maneuver_time = 0
        self.to_steer = 100
        self.img = None # Current camera image
        self.pos = None # Current position (x, y)
        self.yaw = None # Current yaw angle
        self.pitch = None # Current pitch angle
        self.roll = None # Current roll angle
        self.vel = None # Current velocity
        self.steer = 0 # Current steering angle
        self.past_steer = 0 # Track previous steering angle
        self.throttle = 0 # Current throttle value
        self.brake = 0 # Current brake value
        self.nav_angles = None # Angles of navigable terrain pixels
        self.nav_dists = None # Distances of navigable terrain pixels
        self.goal_dists = None
        self.goal_angles = None
        self.ground_truth = ground_truth_3d # Ground truth worldmap
        self.mode = 'forward' # Current mode (can be forward or stop)
        self.throttle_set = 0.4 # Throttle setting when accelerating
        self.brake_set = 10 # Brake setting when braking
        # The stop_forward and go_forward fields below represent total count
        # of navigable terrain pixels.  This is a very crude form of knowing
        # when you can keep going and when you should stop.  Feel free to
        # get creative in adding new fields or modifying these!
        self.stop_forward =20 # Threshold to initiate stopping
        self.go_forward = 450 # Threshold to go forward again
        self.max_vel = 2.5 # Maximum velocity (meters/second)
        # Image output from perception step
        # Update this image to display your intermediate analysis steps
        # on screen in autonomous mode
        self.vision_image = np.zeros((160, 320, 3), dtype=np.float) 
        # Worldmap
        # Update this image with the positions of navigable terrain
        # obstacles and rock samples
        self.worldmap = np.zeros((200, 200, 3), dtype=np.float)
        self.discovered_locs = []
        self.samples_pos = None # To store the actual sample positions
        self.samples_to_find = 0 # To store the initial count of samples
        self.samples_located = 0 # To store number of samples located on map
        self.samples_collected = 0 # To count the number of samples collected
        self.near_sample = 0 # Will be set to telemetry value data["near_sample"]
        self.picking_up = 0 # Will be set to telemetry value data["picking_up"]
        self.send_pickup = False # Set to True to trigger rock pickup
        self.finished = False

#Initialize rover with parameters
Rover = RoverState()


def callback(data):
    frame = np.zeros((500, 500,3), np.uint8)
    angle = data.angle_max
    Vx = 250
    Vy = 250
    for r in data.ranges:
        if r == float ('Inf'):
            r = data.range_max
        x = math.trunc( (r * 10)*math.cos(angle + (-90*3.1416/180)) )
        y = math.trunc( (r * 10)*math.sin(angle + (-90*3.1416/180)) )
        cv2.line(frame,(250, 250),(x+250,y+250),(255,0,0),1)
        Vx+=x
        Vy+=y
        angle= angle - data.angle_increment

    cv2.line(frame,(250, 250),(250+Vx,250+Vy),(0,0,255),3)
    cv2.circle(frame, (250, 250), 2, (255, 255, 0))
    ang = -(math.atan2(Vx,Vy)-3.1416)*180/3.1416
    if ang > 180:
        ang -= 360
    cv2.putText(frame,str(ang)[:10], (50,400), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255))

    cv2.imshow('frame',frame)
    cv2.waitKey(1)
    
    yaw = 1500 + ang * 40 / 6
    throttle = 1900
        
    msg = OverrideRCIn()
    msg.channels[0] = yaw
    msg.channels[1] = 0
    msg.channels[2] = throttle
    msg.channels[3] = 0
    msg.channels[4] = 0
    msg.channels[5] = 0
    msg.channels[6] = 0
    msg.channels[7] = 0
    pub.publish(msg)   


def laser_listener():
    rospy.init_node('navigate', anonymous=True)
    
    # rospy.Subscriber("/scan", LaserScan,callback)
    rospy.Subscriber("/rover/front/image_front_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    laser_listener()