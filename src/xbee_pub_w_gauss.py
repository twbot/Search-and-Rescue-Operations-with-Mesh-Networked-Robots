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
battery = 1
nodes = []
node_rely = None
node_send = None
rssi_rely = 0
rssi_margin_left = 0
rssi_margin_right = 0
rssi_thresh_right = 0
rssi_thresh_left = 0
current_rssi = 0
data = []
rssi_avg = 0
rssi_hist = []
turning_hist = []
data_hist = []
avg_count = 5
rssi_margin = 2
rssi_thresh = 10
vehicle = None
packets_sent = 0
throttle = 0;

#Publisher for rc data (turning angle, throttle)
rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

#Function called at beginning to find nodes and instantiate the location of other
#nodes in the system, relative to one another
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

def determine_architecture():
    if(node_id == 'COORDINATOR'):
        #Grab a node from list of nodes in system, send data, and determine RSSI
        #from received packet
        for node in nodes:
            count = 0
            sending_node = None
            rssi_det = []
            #Average all RSSI values received from determined node
            while count < avg_count:
                xbee.send_data(node,"DATREQ")
                data = None
                time_pass = 0
                start_time = time.time()
                while data == None:
                    packet = xbee.read_data()
                    data = packet
                    #If no packet is received in 6 seconds,
                    #exit program
                    time_pass = check_time(start_time, 6)
                    if(time_pass):
                        rospy.logerr('Could not retreive data from node: ')
                        rospy.logerr(node)
                        return 0
                count = count + 1
                sending_node = data.remote_device
                rssi_det.append(int(data.data.decode()))
            rssi = float(sum(rssi_det)/len(rssi_det))
            #Add node and corresponding RSSI to RSSI table
            init_rssi_table(sending_node, rssi)
        self_node = {}
        self_node["node"] = str(address)
        self_node["rssi"] = 0
        rssi_table.append(self_node)
        #Once all nodes have been determined, along with corresponding RSSI's
        #sort data table by values with lowest absolute valued RSSI
        sort_table_by_rssi()
    else:
        count = 0
        #Receive N data packets from Coordinator
        while count < avg_count:
            data = None
            #Continue checking for packet received
            while data == None:
                packet = xbee.read_data()
                data = packet
            val = data.data.decode()
            sending_node = data.remote_device
            #Send RSSI data back to remote device
            if val == 'DATREQ':
                rssi = get_RSSI()
                string = str(rssi).encode()
                xbee.send_data(sending_node, string)
            count = count + 1
    return 1

def define_node(node):
    node = re.findall(r'[\w\d]+', str(node))
    return node[0]

def send_rssi_table():
    if(node_id == 'COORDINATOR'):
        for node in nodes:
            receive_ack = None
            time_pass = 0
            table = convert_list_to_bytearr()
            xbee.send_data(node, table)
    else:
        data = None
        while data == None:
            packet = xbee.read_data()
            data = packet
        val = data.data
        convert_bytearr_to_list(val)
    return 1

def init_rssi_table(node_sent, rssi):
    sending_node = define_node(node_sent)
    node = {}
    node["node"] = str(sending_node)
    node["rssi"] = rssi
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

def determine_rssi_value():
    if node_send:
        xbee.send_data(node_send,"RSSI_DET")
    if not node_id == 'COORDINATOR':

        data = None
        time_pass = 0
        start_time = time.time()
        rssi_found = 0
        while data == None:
            packet = xbee.read_data()
            data = packet
            time_pass = check_time(start_time, 6)
            if(time_pass):
                rospy.logerr('Could not retreive data from node: ')
                rospy.logerr(node)
                return 0
        rssi = get_RSSI()
        rospy.loginfo("Starting RSSI")
        rospy.loginfo(rssi)
        rospy.loginfo('Data Retrieved')
        sending_node = data.remote_device
        data = data.data.decode()
        if (sending_node == node_rely) and (data == 'RSSI_DET'):
            global rssi_rely
            rssi_rely = rssi

    #Update margin-of-error and thresholding values
    global rssi_margin_right
    rssi_margin_right = rssi_rely+rssi_margin
    global rssi_margin_left
    rssi_margin_left = rssi_rely-rssi_margin
    global rssi_thresh_right
    rssi_thresh_right = rssi_rely+rssi_thresh
    global rssi_thresh_left
    rssi_thresh_left = rssi_rely-rssi_thresh
    
    #Update RSSI history table with current value RSSI
    count = 0
    while count < avg_count:
        rssi_hist.append(rssi_rely)
        count = count + 1
    return 1

def determine_neighbors():
    index = 0
    #Grab self index within RSSI table
    for node in rssi_table:
        if node["node"] == address:
            index = rssi_table.index(node)
    #If first in table, set node_rely to None (Coordinator)
    global node_rely
    if index == 0:
        node_rely = None
    #Else determine node_rely to be index below current index
    else:
        node_val = rssi_table[index-1]
        for node in nodes:
            if node_val["node"] == str(node.get_64bit_addr()):
                node_rely = node
    #If last in table, set node_rely to None
    global node_send
    if index == (len(rssi_table)-1):
        node_send = None
    #Else determine node_rely to be index above current index
    else:
        node_val = rssi_table[index+1]
        for node in nodes:
            if node_val["node"] == str(node.get_64bit_addr()):
                node_send = node
    return 1

def takeoff_copter():
    #After arming copter, set for mode: takeoff
    rospy.wait_for_service('/mavros/cmd/arming')
    arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    response = arming(value=True)
    if not "True" in str(response):
        rospy.logerr('Could not arm copter for takeoff')
        return 0
    rospy.wait_for_service('/mavros/cmd/takeoff')
    takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    response = takeoff(altitude=3, latitude=0, longitude=0, min_pitch=0, yaw=0)
    if not "True" in str(response):
        rospy.logerr('Failed takeoff')
        return 0
    else:
        return 1

def land_copter():
    #Land copter on ros end or keyboard interrupt
    rospy.wait_for_service('/mavros/cmd/land')
    land = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
    response = land(altitude=5, latitude=0, longitude=0, min_pitch=0, yaw=0)
    if not "True" in str(response):
        rospy.logerr('Failed land')
        return 0
    else:
        return 1

def takeoff_rover():
    pass

def determine_RSSI(received):
    if node_rely:
        sending_node = received.remote_device
        throttle = received.data.decode()
        sending_node = define_node(sending_node)
        if sending_node is not None and (sending_node == str(node_rely.get_64bit_addr())):
            rssi = get_RSSI()
            rssi_hist.append(rssi)
        return throttle

def send_ack(throttle):
    if node_send:
        xbee.send_data_async(node_send, throttle)
        global packets_sent
        packets_sent = packets_sent + 1

def coordinate_copter_control():
    # coordinate_copter_velocities()
    pass

def unsigned(n):
    return n & 0xFFFFFFFF

def coordinate_rover_control(throttle):
    yaw = 1500
    scale = 7
    steer_range = 400

    if current_rssi < rssi_margin_right and current_rssi > rssi_margin_left:
        coordinate_rover_velocities(yaw, throttle)
        rospy.logerr("Going Straight")
    else:
        value = current_rssi - rssi_rely
        magnitude = abs(value)
        value_scaled = (magnitude/rssi_thresh)*scale

        def function(x):
            return 0.34*math.pow(x, 2)

        def function2(x):
            return 0.8*math.pow(math.e, x)

        steer_angle = 0
        if value < 0:
            steer_angle = function2(value_scaled)
        elif value > 0:
            steer_angle = -function(value_scaled)
        yaw = 1500+(steer_angle/scale)*steer_range
        if (yaw > 1900):
            yaw = 1900
        elif (yaw < 1100):
            yaw = 1100
        turning_hist.append(unsigned(int(yaw)))
        rospy.loginfo("Yaw")
        rospy.loginfo(yaw)
        coordinate_rover_velocities(yaw, throttle)

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

def coordinate_copter_velocities(roll, pitch, throttle, yaw):
    msg = OverrideRCIn()
    msg.channels[0] = roll
    msg.channels[1] = pitch
    msg.channels[2] = throttle
    msg.channels[3] = yaw
    msg.channels[4] = 0
    msg.channels[5] = 0
    msg.channels[6] = 0
    msg.channels[7] = 0
    rc_pub.publish(msg)   

def determined_path_rover(start_time, throttle):
    sample_time = time.time()
    yaw = 1500
    if ((sample_time - start_time) > 5) and ((sample_time - start_time) < 5.5):
        yaw = 1300
    if ((sample_time - start_time) > 8) and ((sample_time - start_time) < 9):
        yaw = 1700
    if ((sample_time - start_time) > 12) and ((sample_time - start_time) < 13):
        yaw = 1300
    if ((sample_time - start_time) > 13) and ((sample_time - start_time) < 13.9):
        yaw = 1500
    if ((sample_time - start_time) > 15) and ((sample_time - start_time) < 16):
        yaw = 1500
    # if ((sample_time - start_time) > 16.4) and ((sample_time - start_time) < 16.7):
    #     yaw = 1350
    # if ((sample_time - start_time) > 16.7) and ((sample_time - start_time) < 17.4):
    #     yaw = 1750
    # if ((sample_time - start_time) > 17.5) and ((sample_time - start_time) < 18):
    #     yaw = 1350
    # if ((sample_time - start_time) > 18) and ((sample_time - start_time) < 19):
    #     yaw = 1500
    # if ((sample_time - start_time) > 15) and ((sample_time - start_time) < 16):
    #     yaw = 1500
    coordinate_rover_velocities(yaw, throttle)

def battery_callback(battery_data):
    #Recieve percentage parameter from ros publisher
    battery_status = battery_data.percentage
    battery_voltage = battery_data.voltage
    rospy.loginfo(batter_status)
    rospy.loginfo(battery_voltage)
    #If battery status below 10%, change battery bool
    if batter_status < .10:
        battery = 0

def filter_rssi(window, window_gauss):

    last_vals = rssi_hist[:10]
    gauss_vals = []
    filtered_data = []

    def function(x, var, mu):
        if var == 0:
            var = 1
        return (1/(var*math.sqrt(2*math.pi)))*math.pow(math.e, (-math.pow((x-mu), 2))/(2*math.pow(var, 2)))

    def variance_func(val_list, avg):
        value = 0
        for val in val_list:
            value = value + math.pow((val-avg), 2)
        value = value/(len(val_list)-1)
        return value

    index = 0
    for value in rssi_hist:
        avg = sum(rssi_hist[-window:])/window
        variance = variance_func(rssi_hist[-window:], avg)
        value = function(rssi_hist, variance, avg)
        if index == 0:
            value_list = [value]*13
            gauss_vals = gauss_vals + value_list
        gauss_vals.append(value)
        avg_gauss_vals = sum(gauss_vals[-window_gauss:])/window_gauss
        count = 0
        if value > avg_gauss_vals:
            count = count+1
            val_tuple = (value[0], value[1])
            filtered_data.append(val_tuple)
        last_vals.append(row)
        index = index+1
    return 

def get_RSSI():
    rssi = xbee.get_parameter("DB")
    rssi = struct.unpack("=B", rssi)
    return rssi[0]

def check_time(start_time, wanted_time):
    current_time = time.time()
    if((current_time - start_time) > wanted_time):
        return 1
    return 0

def send_data_to_file(data):
    directory = os.getcwd()
    file = os.path.join(directory, 'turn_radius.csv')
    # data_file = pd.DataFrame(data)
    # data_file = data_file.to_csv(file, index=False, header=False)

def on_end():
    if xbee is not None and xbee.is_open():
        xbee.close()
        print('Xbee Closed')
    # if vehicle == 'Copter':
        # land_copter()
    print(rssi_hist)
    # print(turning_hist)
    print(data_hist)

def main(vehicle_type, velocity, threshold):
    #Set global variables for:
    # - vehicle throttle (based on coordinator's throttle)
    # - vehicle type
    # - RSSI thresholding
    global throttle
    throttle = velocity
    global vehicle
    vehicle = vehicle_type
    global rssi_margin
    rssi_margin = threshold

    rospy.init_node('Search_Run')
    r = rospy.Rate(30)
    mission_complete = 0

    net_instantiated = instantiate_zigbee_network()
    arch_instantiated = determine_architecture()
    print('Net Instantiated') if net_instantiated else print('Net failed to instantiated')
    print('Architecture Instantiated') if arch_instantiated else print('Architecture failed to instantiate')
    
    init_complete = 0
    if arch_instantiated and net_instantiated:
        init_complete = send_rssi_table()
    
    for x in rssi_table:
        print(x["node"], " : ", x["rssi"])
    
    determined_neighbors = 0
    if init_complete:
        determined_neighbors = determine_neighbors()

    rssi_determined = 0
    if determined_neighbors:
        rssi_determined = determine_rssi_value()

    print("Node rely: ", node_rely)
    print("Node send: ", node_send)
    
    # if node_id == 'COORDINATOR' and vehicle == 'Copter':
       # takeoff_copter()

    if rssi_determined:
        exec_time = 30
        mission_start_time = time.time()
        while (not rospy.is_shutdown()) and (not mission_complete):
            mission_complete = check_time(mission_start_time, exec_time)
            global throttle
            send_ack(throttle)
            received = xbee.read_data()
            if received:
                throttle = determine_RSSI(received)
            global current_rssi
            current_rssi = filter_rssi(13, 3, rssi)
            current_rssi = float(sum(rssi_hist[-4:])/len(rssi_hist[-4:]))
            rospy.Subscriber("/mavros/battery", BatteryStatus, battery_callback)
            rospy.loginfo("RSSI Val: ")
            rospy.loginfo(current_rssi)
            rospy.loginfo("Throttle: ")
            rospy.loginfo(throttle)
            # if vehicle == 'Copter' and node_id != 'COORDINATOR':
               # coordinate_copter_control()
            # if vehicle == 'Copter' and node_id == 'COORDINATOR':
               # determined_path_copter(mission_start_time, throttle)
            if vehicle == 'Rover' and node_id != 'COORDINATOR':
                coordinate_rover_control(throttle)
            if vehicle == 'Rover' and node_id == 'COORDINATOR':
                determined_path_rover(mission_start_time, throttle)
            r.sleep()
    
    else:
        on_end()

    rospy.on_shutdown(on_end)
    
if __name__ == '__main__':

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('vehicle_type', help='Type of Vehicle: Copter or Rover', choices=['Rover', 'Copter'],  default='Rover')
    parser.add_argument('init_velocity', help='Initial velocity of vehicles', nargs='?', default=1650)
    parser.add_argument('threshold', help='Threshold for RSSI')
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
            main(args.vehicle_type, args.init_velocity, args.threshold)
        except rospy.ROSInterruptException:
            rospy.logerr("Problem changing operating mode")
            pass
