#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math

import xbee

from mavros_msgs.msg import OverrideRCIn, BatteryStatus
from node_msg import NodeStatus

from digi.xbee.devices import ZigBeeDevice
from digi.xbee.packets.base import DictKeys

sysyem_nodes = {}

xbee = ZigBeeDevice("/dev/ttyUSB0", 9600)
power_level = 1
api_mode = 2
hierarchy = 0
node_id = ''

node_pub = rospy.Publisher('/node_status', NodeStatus, queue_size=10)

def instantiate_zigbee_network():
    try:
        xbee.open()
        xbee.set_parameter('PL', utils.int_to_bytes(power_level, num_bytes=1))
        xbee.set_parameter('AP', utils.int_to_bytes(api_mode, num_bytes=1))
        node_id = xbee.get_node_id()
        print("This Node ID: ", node_id)
        print("Is Remote: ", xbee.is_remote())
        print("Power Level: ", xbee.get_power_level())
        hierarchy = xbee.get_parameter('CE')
        print(hierarchy)
        if not hierarchy:
            router_val = xbee.get_node_id()
            router_val = router_val.split('_')[1]
            hierarchy = router_val

        print("Entering discovery mode...\n")

        xnet = xbee.get_network()
        xnet.set_discovery_timeout(15)
        xnet.clear()

        xnet.add_device_discovered_callback(xbee.discoverCallback)
        xnet.add_discovery_process_finished_callback(xbee.discoverCompleteCallback)
        xnet.start_discovery_process()

        while xnet.is_discovery_running():
            time.sleep(0.5)

        nodes = xnet.get_devices()

        data = 'Zigbee node %s sending data' % (xbee.get_node_id())

        for node in nodes:
            print("Nodes found: %s" % node)
            data = data.encode('utf-8')
            rssi_raw = xbee.get_parameter('DB')
            rssi_val = struct.unpack('=B', rssi_raw)
            print("Node RSSI: %s" % rssi_val)

        data = data.encode('utf-8')
        rssi_raw = xbee.get_parameter('DB')
        rssi_val = struct.unpack('=B', rssi_raw)
        print("Node RSSI: %s" % rssi_val)
        print("Node RSSI: %s" % rssi_raw)

    except ZigBeeException:
        print('Error %s' % ZigBeeException)

def check_mission_status():
    mission_status = False

def node_callback(battery_data):
    batter_status = battery_data.remaining
    #Mission status will be None for now, until 
    mission_status = None

def node_data_publisher():
    rospy.init_node('node_status', anonymous=True)
    recieved = xbee.add_packet_received_callback(xbee.packet_received_callback)
    rospy.Subscriber('/mavros/battery', BatteryStatus, node_callback)

    if recieved:
        node_pub.publish(data)

def on_end():
    if xbee is not None and xbee.is_open():
        xbee.close()

def main():
    rospy.init_node('node_status')
    recieved = xbee.add_packet_received_callback(xbee.packet_received_callback)
    
    
    instantiate_zigbee_network()
    node_data_publisher()
    # rate = rospy.Rate(10)

    while not rospy.is_shutdown() or mission_status:
        check_mission_status()
        rospy.spin()

    rospy.on_shutdown(on_end)
    
if __name__ == '__main__':
    main()