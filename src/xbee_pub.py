#!/usr/bin/env python3

import rospy
#import cv2
#import numpy as np
import math
import xbee as xlib
import time
import struct 

from mavros_msgs.msg import OverrideRCIn, BatteryStatus
#from msg import NodeStatus

from digi.xbee.devices import ZigBeeDevice
from digi.xbee.packets.base import DictKeys
from digi.xbee.exception import XBeeException, ConnectionException, ATCommandException, InvalidOperatingModeException
from digi.xbee.util import utils

sysyem_nodes = {}

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

        data = 'Zigbee node %s sending data' % (xbee.get_node_id())

        for node in nodes:
            print("Nodes found: %s" % node)
            #data = str(data)
            #data = data.encode('utf-8')
            rssi_raw = xbee.get_parameter('DB')
            rssi_val = struct.unpack('=B', rssi_raw)
            print("Node RSSI: %s" % rssi_val)

        #data = data.encode('utf-8')
        #rssi_raw = xbee.get_parameter('DB')
        #rssi_val = struct.unpack('=B', rssi_raw)
        #print("Node RSSI: %s" % rssi_val)
        #print("Node RSSI: %s" % rssi_raw)
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
            not_read = None
            while not_read == None:
                packet = xbee.read_data()
                if packet is not None:
                    packet = packet.to_dict()
                    data = packet[DictKeys.FRAME_SPEC_DATA][DictKeys.API_DATA]
                    rssi = xbee.get_parameter("DB")
                    rssi = struct.unpack('=B', rssi)
                not_read = packet
    else:
        not_read = None
        while not_read == None:
            packet = xbee.read_data()
            if packet is not None:
                packet = packet.to_dict()
                data = packet[DictKeys.FRAME_SPEC_DATA][DictKeys.API_DATA]
                #sent_node_id = struct.unpack('=B', package.remote_device)
                #if val == 'DATAREQ': 
                #    xbee.send_data(sent_node_id, node_id) 
                print(data)
            not_read = packet
    print('Architecture Determined')
    return 1

def update_rssi_table(packet):
    print(packet)

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
    #rate = rospy.Rate(10)
    print('Net Instantiated: ', net_instantiated)
    print('Arch Instantiated: ', arch_instantiated)
    #if net_instantiated and arch_instantiated:
       # while (not rospy.is_shutdown()) or mission_status:
       #     mission_status = check_mission_status()
       #     received_packet = xbee.add_packet_received_callback(xbee.packet_received_callback)
       #     print(received_packet)
    #        update_rssi_table(received_packet)
            # rospy.Subscriber("/mavros/battery", BatteryStatus, update_rssi_table)
            # coordinate_velocities()
       #     rospy.spin()
    
    #else:
    #    rospy.on_shutdown(on_end)

    rospy.on_shutdown(on_end)
    
if __name__ == '__main__':
    main()
