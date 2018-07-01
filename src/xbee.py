#!/usr/bin/env python3

from digi.xbee.devices import XBeeDevice, ZigBeeDevice
from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.packets.base import DictKeys
import time

xbee = ZigBeeDevice("/dev/ttyUSB0", 9600)

try:
    xbee.open()
    xbee
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
    
    for device in devices:
        print("Devices found: %s" % device)

    def get_packet_recieved_callback(packet):
        packet_dict = packet.to_dict()
        api_data = packet_dict[DictKeys.FRAME_SPEC_DATA][DictKeys.API_DATA]
            
    def initialize_nodes():
	for node in nodes:
		
		 
finally:
    if xbee is not None and xbee.is_open():
        xbee.close()
