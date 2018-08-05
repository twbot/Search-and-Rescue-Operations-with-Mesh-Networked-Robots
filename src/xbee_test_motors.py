#!/usr/bin/env python3

import rospy
#import cv2
#import numpy as np
import math
import xbee as xlib
import time
import struct
import re
import argparse
import math
import csv
import os

from mavros_msgs.msg import OverrideRCIn, BatteryStatus
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
#from msg import NodeStatus
from digi.xbee.models.address import XBee64BitAddress
from digi.xbee.devices import ZigBeeDevice
from digi.xbee.packets.base import DictKeys
from digi.xbee.exception import XBeeException, ConnectionException, ATCommandException, InvalidOperatingModeException
from digi.xbee.util import utils
from digi.xbee.io import IOLine, IOMode


rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

def coordinate_rover_velocities(yaw, throttle):
    yaw = unsigned(int(yaw))
    value = (yaw, current_rssi)
    data_hist.append(value)
    throttle = unsigned(int(throttle))
    msg = OverrideRCIn()
    msg.channels[0] = yaw
    msg.channels[1] = 0
    msg.channels[2] = throttle
    msg.channels[3] = 0
    msg.channels[4] = 0
    msg.channels[5] = 0
    msg.channels[6] = 0
    msg.channels[7] = 0
    rc_pub.publish(msg)   

def determined_path_rover(start_time, throttle):
    sample_time = time.time()
    yaw = 1500
    if ((sample_time - start_time) > 3) and ((sample_time - start_time) < 4):
        yaw = 1100
    if ((sample_time - start_time) > 6) and ((sample_time - start_time) < 7):
        yaw = 1900
    if ((sample_time - start_time) > 10) and ((sample_time - start_time) < 11):
        yaw = 1200
    if ((sample_time - start_time) > 11) and ((sample_time - start_time) < 11.4):
        yaw = 1900
    if ((sample_time - start_time) > 11.5) and ((sample_time - start_time) < 12):
        yaw = 1500

    # if ((sample_time - start_time) > 3) and ((sample_time - start_time) < 4):
    #     yaw = 1100
    # if ((sample_time - start_time) > 6) and ((sample_time - start_time) < 7):
    #     yaw = 1900
    # if ((sample_time - start_time) > 10) and ((sample_time - start_time) < 11):
    #     yaw = 1200
    # if ((sample_time - start_time) > 11) and ((sample_time - start_time) < 11.4):
    #     yaw = 1900
    # if ((sample_time - start_time) > 11.5) and ((sample_time - start_time) < 12):
    #     yaw = 1500

    # if ((sample_time - start_time) > 3) and ((sample_time - start_time) < 4):
    #     yaw = 1100
    # if ((sample_time - start_time) > 6) and ((sample_time - start_time) < 7):
    #     yaw = 1900
    # if ((sample_time - start_time) > 10) and ((sample_time - start_time) < 11):
    #     yaw = 1200
    # if ((sample_time - start_time) > 11) and ((sample_time - start_time) < 11.4):
    #     yaw = 1900
    # if ((sample_time - start_time) > 11.5) and ((sample_time - start_time) < 12):
    #     yaw = 1500

    coordinate_rover_velocities(yaw, throttle)

def main(vehicle_type, velocity, data_send):
    global throttle
    throttle = velocity
    global vehicle
    vehicle = vehicle_type

    rospy.init_node('Search_Run')
    r = rospy.Rate(30)
    mission_complete = 0

    exec_time = 30
    mission_start_time = time.time()
    while (not rospy.is_shutdown()) and (not mission_complete):
        mission_complete = check_time(mission_start_time, exec_time)
        global throttle
        rospy.loginfo("Throttle: ")
        rospy.loginfo(throttle)
        if vehicle == 'Rover':
            determined_path_rover(mission_start_time, throttle)
        r.sleep()
    
    else:
        on_end()

    rospy.on_shutdown(on_end)