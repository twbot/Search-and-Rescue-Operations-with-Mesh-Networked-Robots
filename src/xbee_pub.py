#!/usr/bin/env python3

import rospy
#import cv2
#import numpy as np
import math
import xbee as xlib
import time
import struct 
import re

from mavros_msgs.msg import OverrideRCIn, BatteryStatus
#from msg import NodeStatus

from digi.xbee.devices import ZigBeeDevice
from digi.xbee.packets.base import DictKeys
from digi.xbee.exception import XBeeException, ConnectionException, ATCommandException, InvalidOperatingModeException
from digi.xbee.util import utils
from digi.xbee.io import IOLine, IOMode

system_nodes = {}
ordered_nodes = []

xbee = ZigBeeDevice("/dev/ttyUSB0", 9600)
power_level = 1
api_mode = 2
hierarchy = 0
node_id = ''
received_packet = None
battery = 1
mission_time = 0
exec_time = 15
nodes = []

pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

def instantiate_zigbee_network():
    try:
        print("Opening xbee port")
        xbee.open()
        print("Setting Power Level")
        xbee.set_parameter('PL', utils.int_to_bytes(power_level, num_bytes=1))
        print("Setting API Mode")
        xbee.set_parameter('AP', utils.int_to_bytes(api_mode, num_bytes=1))
        xbee.set_io_configuration(IOLine.DIO4_AD4, IOMode.DISABLED)
        print("Getting self id")
        global node_id
        node_id = xbee.get_node_id()
        print("This Node ID: ", node_id)
        print("Is Remote: ", xbee.is_remote())
        print("Power Level: ", xbee.get_power_level())

        print("Entering discovery mode...\n")

        xnet = xbee.get_network()
        xnet.set_discovery_timeout(15)
        xnet.clear()

        xnet.add_device_discovered_callback(xlib.discoverCallback)
        xnet.add_discovery_process_finished_callback(xlib.discoverCompleteCallback)
        xnet.start_discovery_process()

        while xnet.is_discovery_running():
            time.sleep(0.5)
        global nodes
        nodes = xnet.get_devices()
        for node in nodes:
            print(node)
            print(type(node))
        data = 'Zigbee node %s sending data' % (xbee.get_node_id())

        return 1

    except ConnectionException:
        print('Error Connection')
        xbee.close()
        return 0
    except ATCommandException:
        print('Response of the command is not valid : ATCommandException')
        xbee.close()
        return 0
    except InvalidOperatingModeException:
        print('Not in API Mode')
        xbee.close()
        return 0

def determine_architecture():
    if(node_id == 'COORDINATOR'):
        for node in nodes:
            xbee.send_data(node,"DATREQ")
            print('Data sent')
            data = None
            while data  == None:
                packet = xbee.read_data()
                data = packet
            update_rssi_table(packet)
    else:
        data = None
        while data == None:
            packet = xbee.read_data()
            data = packet
        val = data.data.decode()
        sending_node = data.remote_device
        if val == 'DATREQ': 
            xbee.send_data(sending_node, node_id) 
        print("Value: ", val)
        print("Sending node: ", sending_node)
    print('Architecture Determined')
    return 1

def define_node(node):
    node = re.findall(r'\d+', node)
    print(node)
    return node

def init_rssi_table():
    ordered_nodes.sort(key=lambda tup: tup[1])
    for node in ordered_nodes:
        print(node)

def update_rssi_table(packet):
    val = packet.data.decode()
    sending_node = packet.remote_device
    sending_node = define_node(sending_node)
    rssi = xbee.get_parameter("DB")
    rssi = struct.unpack("=B", rssi)
    system_nodes[str(sending_node)] = rssi[0]

def coordinate_velocities():
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

def battery_callback():
    #Recieve percentage parameter from ros publisher
    battery_status = battery_data.percentage
    #If battery status below 10%, change battery bool
    if batter_status < .10:
        battery = 0

def check_mission_status():
    current_time = time.time()
    if((current_time - mission_time) > exec_time):
        return 1
    return 0

def node_callback(battery_data):
    batter_status = battery_data.remaining
    #If battery falls below 10% capacity, notify recipients
    #Mission status: time remaining to travel

def on_end():
    if xbee is not None and xbee.is_open():
        xbee.close()
        print('Xbee Closed')

def main():
    rospy.init_node('node_status')
    mission_time = time.time()
    mission_status = 0
    net_instantiated = instantiate_zigbee_network()
    arch_instantiated = determine_architecture()
    #init_rssi_table()
    #rate = rospy.Rate(10)
    print('Net Instantiated: ', net_instantiated)
    print('Arch Instantiated: ', arch_instantiated)
    for x in system_nodes:
        print(x, " : ", system_nodes[x])
        
    #if net_instantiated and arch_instantiated:
        #while (not rospy.is_shutdown()) or mission_status:
            #mission_status = check_mission_status()
            #rospy.Subscriber("/mavros/battery", BatteryStatus, update_status)
            #coordinate_velocities()
            #rospy.spin()
    
    #else:
        #rospy.on_shutdown(on_end)

    rospy.on_shutdown(on_end)
    
if __name__ == '__main__':
    main()
