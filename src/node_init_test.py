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

system_nodes = {}
rssi_table = []

xbee = ZigBeeDevice("/dev/ttyUSB0", 9600)
power_level = 2
api_mode = 2
hierarchy = 0
node_id = ''
address = ''
received_packet = None
nodes = []
data_hist = []

rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

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
        global address
        address = str(xbee.get_64bit_addr())
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

def define_node(node):
    node = re.findall(r'[\w\d]+', str(node))
    return node[0]

def convert_list_to_bytearr():
    encoded_val = []
    for node in rssi_table:
        id = node["node"]
        rssi = str(node["rssi"])
        val = (id, rssi)
        val = '_'.join(val)
        encoded_val.append(val)
    encoded_val = ':'.join(encoded_val)
    encoded_val = encoded_val.encode()
    return encoded_val

def convert_bytearr_to_list(bytearr):
    data_list = bytearr.decode()
    data_list = data_list.split(':')
    for data in data_list:
        data = data.split('_')
        node = {}
        node["node"] = data[0]
        node["rssi"] = data[1]
        rssi_table.append(node)

def get_RSSI():
    rssi = xbee.get_parameter("DB")
    rssi = struct.unpack("=B", rssi)
    return rssi[0]

def on_end():
    if xbee is not None and xbee.is_open():
        xbee.close()
        print('Xbee Closed')
    print(data_hist)

def check_time(start_time, wanted_time):
    current_time = time.time()
    if((current_time - start_time) > wanted_time):
        return 1
    return 0

def send_packet():
    for node in nodes:
        xbee.send_data_async(node, 'ACK')

def append_data():
    rssi = get_RSSI()
    curr_time = time.time()
    value = (rssi, curr_time)
    data_hist.append(value)

def main():

    net_instantiated = instantiate_zigbee_network()
    'Net Instantiated' if net_instantiated else 'Net failed to instantiated'
    'Architecture Instantiated' if arch_instantiated else 'Architecture failed to instantiate'

    exec_time = 30
    mission_start_time = time.time()
    mission_complete = 0
    print('Time Start: ', mission_start_time)

    while (not mission_complete) and net_instantiated:
        mission_complete = check_time(mission_start_time, exec_time)
        send_packet()
        append_data()

    on_end()
    
if __name__ == '__main__':
    main()