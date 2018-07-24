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

from mavros_msgs.msg import OverrideRCIn, BatteryStatus
from mavros_msgs.srv import SetMode
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
power_level = 1
api_mode = 2
hierarchy = 0
node_id = ''
address = ''
received_packet = None
battery = 1
nodes = []
node_rely = None
node_send = None
rssi_rely = 0
 
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
        print(type(nodes[1]))
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
            time_pass = 0
            start_time = time.time()
            while data  == None:
                packet = xbee.read_data()
                data = packet
                time_pass = check_time(start_time, 6)
                if(time_pass):
                    print('Could not retreive data from node: ', node)
                    return 0
            init_rssi_table(data)
        self_node = {}
        self_node["node"] = str(address)
        self_node["rssi"] = 0
        rssi_table.append(self_node)
        sort_table_by_rssi()
    else:
        data = None
        while data == None:
            packet = xbee.read_data()
            data = packet
        val = data.data.decode()
        sending_node = data.remote_device
        if val == 'DATREQ': 
            xbee.send_data(sending_node, node_id)
    return 1

def define_node(node):
    node = re.findall(r'[\w\d]+', str(node))
    return node[0]

def get_RSSI():
    rssi = xbee.get_parameter("DB")
    rssi = struct.unpack("=B", rssi)
    return rssi[0]

def send_rssi_table():
    if(node_id == 'COORDINATOR'):
        for node in nodes:
            receive_ack = None
            time_pass = 0
            #start_time = time.time()
            table = convert_list_to_bytearr()
            xbee.send_data(node, table)
            print('RSSI Table send to: ', node)
    else:
        data = None
        while data == None:
            packet = xbee.read_data()
            data = packet
        val = data.data
        convert_bytearr_to_list(val)
    return 1

def init_rssi_table(packet):
    val = packet.data.decode()
    sending_node = packet.remote_device
    sending_node = define_node(sending_node)
    node = {}
    node["node"] = str(sending_node)
    node["rssi"] = get_RSSI()
    rssi_table.append(node)

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

def sort_table_by_rssi():
    rssi_table.sort(key=lambda val: val["rssi"])

def determine_neighbors():
    index = 0
    for node in rssi_table:
        if node["node"] == address:
            index = rssi_table.index(node)
    global node_rely
    if index == 0:
        node_rely = None
    else:
        node_val = rssi_table[index-1]
        for node in nodes:
            if node_val["node"] == str(node.get_64bit_addr()):
                node_rely = node
    global node_send
    if index == (len(rssi_table)-1):
        node_send = None
    else:
        node_val = rssi_table[index+1]
        for node in nodes:
            if node_val["node"] == str(node.get_64bit_addr()):
                node_send = node
    return 1
    
def takeoff_copter():
    pass

def takeoff_rover():
    pass

def determine_RSSI(received):
    if node_rely:
        node = define_node(received)
        if node == str(node_rely.get_64bit_addr()):
            global rssi_rely
            rssi_rely = get_RSSI()
            print(rssi_rely)

def send_ack():
    if node_send:
        xbee.send_data_async(node_send, address)
        print('Sent Ack')

def coordinate_copter_control():
    pass

def coordinate_rover_control():
    pass

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

def battery_callback(battery_data):
    #Recieve percentage parameter from ros publisher
    battery_status = battery_data.percentage
    #If battery status below 10%, change battery bool
    if batter_status < .10:
        battery = 0

def check_time(start_time, wanted_time):
    current_time = time.time()
    if((current_time - start_time) > wanted_time):
        return 1
    return 0

def on_end():
    if xbee is not None and xbee.is_open():
        xbee.close()
        print('Xbee Closed')

def main(vehicle_type, velocity):
    throttle = velocity
    vehicle = vehicle_type

    rospy.init_node('node_status')
    r = rospy.Rate(10)
    mission_status = 0

    net_instantiated = instantiate_zigbee_network()
    arch_instantiated = determine_architecture()
    'Net Instantiated' if net_instantiated else 'Net failed to instantiated'
    'Architecture Instantiated' if arch_instantiated else 'Architecture failed to instantiate'
    init_complete = 0
    if arch_instantiated and net_instantiated:
        init_complete = send_rssi_table()
    
    for x in rssi_table:
        print(x["node"], " : ", x["rssi"])
    
    determined_neighbors = 0
    if init_complete:
        determined_neighbors = determine_neighbors()

    print("Node rely: ", node_rely)
    print("Node send: ", node_send)
    
    #if node_id == 'COORDINATOR' and vehicle == 'Copter':
    #    takeoff_copter()
    
    #if node_id == 'COORDINATOR' and vehicle == 'Rover':
    #    takeoff_rover()

    if determined_neighbors:
        exec_time = 15
        mission_start_time = time.time()
        while (not rospy.is_shutdown()) or mission_status:
            mission_status = check_time(mission_start_time, exec_time)
            send_ack()
            received = xbee.add_data_received_callback(xlib.data_received_callback)
            determine_RSSI(received)
            rospy.Subscriber("/mavros/battery", BatteryStatus, battery_callback)
            #if vehicle == 'Copter':
            #    coordinate_copter_control()
            #elif vehicle == 'Rover':
            #    coordinate_rover_control()
            r.sleep()
    
    else:
        on_end()

    rospy.on_shutdown(on_end)
    
if __name__ == '__main__':

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('vehicle_type', help='Type of Vehicle: Copter or Rover', choices=['Rover', 'Copter'],  default='Rover')
    parser.add_argument('init_velocity', help='Initial velocity of vehicles', default=1560)
    args = parser.parse_args()

    response = None
    if(args.vehicle_type == 'Copter'):
        rospy.wait_for_service('/mavros/set_mode')
        change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = change_mode(custom_mode="guided")
        print(response)
    elif(args.vehicle_type == 'Rover'):
        rospy.wait_for_service('/mavros/set_mode')
        change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = change_mode(custom_mode="manual")
        print(response)
    
    if "True" in str(response):
        try:
            main(args.vehicle_type, args.init_velocity)
        except rospy.ROSInterruptException:
            print("Problem changing operating mode")
            pass
