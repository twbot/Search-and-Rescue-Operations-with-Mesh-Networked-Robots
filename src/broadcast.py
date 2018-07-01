#!/usr/bin/env python3

from digi.xbee.devices import ZigBeeDevice
from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.packets.base import DictKeys, OperatingMode
import digi.xbee.packets.base as packets
import time
import struct

def main():
    zigbee = ZigBeeDevice('/dev/ttyUSB0', 9600)

    try:
        zigbee.open()
        data = 'Zigbee node %s sending data' % (zigbee.get_node_id())
        data = data.encode('utf-8')
        rssi_raw = zigbee.get_parameter("DB")
        rssi_val = struct.unpack('=B', rssi_raw)
        print(rssi_val)
        #packet_receive = zigbee.send_data_broadcast(data)
        #data_packet = packets.XBeePacket()
        #packet = data_packet.create_packet(data, OperatingMode.API_MODE)
        #packet_receive = zigbee.send_packet(packet)

        def packet_received_callback(packet):
            packet_dict = packet.to_dict()
            api_data = packet_dict[DictKeys.FRAME_SPEC_DATA][DictKeys.API_DATA]
            print(api_data)
        
        print(packet_receive)
        #zigbee.add_packet_received_callback(packet_received_callback)


    finally:
        if zigbee is not None and zigbee.is_open():
            zigbee.close()




if __name__ == '__main__':
    main()
