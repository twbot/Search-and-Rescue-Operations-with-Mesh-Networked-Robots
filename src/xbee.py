#!/usr/bin/env python3

from digi.xbee.devices import XBeeDevice, ZigBeeDevice
from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.packets.base import DictKeys, OperatingMode
import digi.xbee.packets.base as packets
import time
import struct


# def main():
#     zigbee = ZigBeeDevice('/dev/ttyUSB0', 9600)

#     try:
#         zigbee.open()
#         data = 'Zigbee node %s sending data' % (zigbee.get_node_id())
#         data = data.encode('utf-8')
#         rssi_raw = zigbee.get_parameter("DB")
#         rssi_val = struct.unpack('=B', rssi_raw)
#         print(rssi_val)
        #packet_receive = zigbee.send_data_broadcast(data)
        #data_packet = packets.XBeePacket()
        #packet = data_packet.create_packet(data, OperatingMode.API_MODE)
        #packet_receive = zigbee.send_packet(packet)

xbee = ZigBeeDevice("/dev/ttyUSB0", 9600)

try:
    xbee.open()
    print("This Node ID: ", xbee.get_node_id())
    print("Is Remote: ", xbee.is_remote())
    print("Power Level: ", xbee.get_power_level())

    print("Entering discovery mode \n")

    xnet = xbee.get_network()

    xnet.set_discovery_timeout(15)

    xnet.clear()
    
    def discoverCallback(remote):
        print("Device discovered: %s" % remote)

    def discoverCompleteCallback(status):
        if status == NetworkDiscoveryStatus.ERROR_READ_TIMEOUT:
            print("Error recieving devices: %s" % status.description)
        elif status == NetworkDiscoveryStatus.SUCCESS:
            print("Discovery process complete")

    xnet.add_device_discovered_callback(discoverCallback)
    xnet.add_discovery_process_finished_callback(discoverCompleteCallback)

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

    def packet_received_callback(packet):
        packet_dict = packet.to_dict()
        api_data = packet_dict[DictKeys.FRAME_SPEC_DATA][DictKeys.API_DATA]
        print(api_data)
    
    # print(packet_receive)
    # zigbee.add_packet_received_callback(packet_received_callback)
		
		 
finally:
    if xbee is not None and xbee.is_open():
        xbee.close()
