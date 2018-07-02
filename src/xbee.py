#!/usr/bin/env python3

from digi.xbee.devices import XBeeDevice, ZigBeeDevice
from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.packets.base import DictKeys, OperatingMode
import digi.xbee.packets.base as packets
from digi.xbee.util import utils
import time
import struct

xbee = ZigBeeDevice("/dev/ttyUSB0", 9600)
power_level = 1
api_mode = 2

try:
    xbee.open()
    xbee.set_parameter('PL', utils.int_to_bytes(power_level, num_bytes=1))
    xbee.set_parameter('AP', utils.int_to_bytes(api_mode, num_bytes=1))
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

    data = data.encode('utf-8')
    rssi_raw = xbee.get_parameter('DB')
    rssi_val = struct.unpack('=B', rssi_raw)
    print("Node RSSI: %s" % rssi_val)
    print("Node RSSI: %s" % rssi_raw)
    def packet_received_callback(packet):
        packet_dict = packet.to_dict()
        api_data = packet_dict[DictKeys.FRAME_SPEC_DATA][DictKeys.API_DATA]
        print(api_data)
    
    # def node_update_position_estimates():
        
    xbee.add_packet_received_callback(packet_received_callback)
		
		 
finally:
    if xbee is not None and xbee.is_open():
        xbee.close()
