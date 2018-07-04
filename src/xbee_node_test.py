#!/usr/bin/env python3

import rospy

import cv2
import numpy as np
import math

import xbee

from sensor_msgs.msg import Image
from mavros_msgs.msg import OverrideRCIn
from node_msg import NodeStatus

from digi.xbee.devices import ZigBeeDevice
from digi.xbee.packets.base import DictKeys


sysyem_nodes = {}

xbee = ZigBeeDevice("/dev/ttyUSB0", 9600)
power_level = 1
api_mode = 2
hierarchy = 0

rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
node_pub = rospy.Publisher('/node_status', NodeStatus, queue_size=10)

def instantiate_zigbee_network():
    try:
        xbee.open()
        xbee.set_parameter('PL', utils.int_to_bytes(power_level, num_bytes=1))
        xbee.set_parameter('AP', utils.int_to_bytes(api_mode, num_bytes=1))
        print("This Node ID: ", xbee.get_node_id())
        print("Is Remote: ", xbee.is_remote())
        print("Power Level: ", xbee.get_power_level())
        hierarchy = xbee.get_parameter('CE')
        print(hierarchy)
        if(!hierarchy){
            router_val = xbee.get_node_id()
            router_val = router_val.split('_')[1]
            hierarchy = router_val
        }

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


def node_data_publisher():
    rospy.init_node('node_status', anonymous=True)
    recieved = xbee.add_packet_received_callback(xbee.packet_received_callback)
    #DETERMINE DATA TO SEND
    if(recieved){
        node_pub.publish(data)
    }
    # rospy.Subscriber("/")
    rospy.spin()

def image_subscriber():
    rospy.init_node('img_node', anonymous=True)
    rospy.Subscriber('/rover/img_data', Image, img_callback)
    rospy.spin()

def on_end():
    if xbee is not None and xbee.is_open():
        xbee.close()

def main():

    instantiate_zigbee_network()
    node_data_publisher()


if __name__ == '__main__':
    main()