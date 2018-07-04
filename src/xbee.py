#!/usr/bin/env python3

from digi.xbee.devices import XBeeDevice, ZigBeeDevice
from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.packets.base import DictKeys, OperatingMode
import digi.xbee.packets.base as packets

def discoverCallback(remote):
    print("Device discovered: %s" % remote)

def discoverCompleteCallback(status):
    if status == NetworkDiscoveryStatus.ERROR_READ_TIMEOUT:
        print("Error recieving devices: %s" % status.description)
    elif status == NetworkDiscoveryStatus.SUCCESS:
        print("Discovery process complete")

def packet_received_callback(packet):
    packet_dict = packet.to_dict()
    api_data = packet_dict[DictKeys.FRAME_SPEC_DATA][DictKeys.API_DATA]
    print(api_data)