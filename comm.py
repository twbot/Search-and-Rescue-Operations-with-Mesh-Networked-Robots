#!/usr/bin/env python3
import time

from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.devices import ZigBeeDevice

def main():

    print(" +---------------------------------------------+")
    print(" | XBee Python Library Discover Devices Sample |")
    print(" +---------------------------------------------+\n")

    zigbee = ZigBeeDevice('/dev/ttyA0', 9600)

    try:
        zigbee.open()

        zigbee_network = zigbee.get_network()

        zigbee_network.set_discovery_timeout(15)  # 15 seconds.

        zigbee_network.clear()

        # Callback for discovered devices.
        def callback_device_discovered(remote):
            print("Device discovered: %s" % remote)

        # Callback for discovery finished.
        def callback_discovery_finished(status):
            if status == NetworkDiscoveryStatus.SUCCESS:
                print("Discovery process finished successfully.")
            else:
                print("There was an error discovering devices: %s" % status.description)

        zigbee_network.add_device_discovered_callback(callback_device_discovered)

        zigbee_network.add_discovery_process_finished_callback(callback_discovery_finished)

        zigbee_network.start_discovery_process()

        print("Discovering remote XBee devices...")

        devices = zigbee_network.get_devices()

        while zigbee_network.is_discovery_running():
            time.sleep(0.1)

    finally:
        if zigbee is not None and zigbee.is_open():
            zigbee.close()


if __name__ == '__main__':
    main()


