#!/usr/bin/env python3

import argparse

def main(current_rssi, rssi_rely):
    yaw = 1500
    scale = 5
    steer_range = 400
    rssi_thresh = 15

    value = current_rssi - rssi_rely
    magnitude = abs(value)
    value_scaled = (magnitude/rssi_thresh)*scale

    def function(x):
        return  0.34*math.pow(x, 2)

    def function2(x):
        return  0.8*math.pow(math.e, x)

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
    print(yaw)

if __name__ == '__main__':
     # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('current_rssi', type=float)
    parser.add_argument('rssi_rely', type=float)
    args = parser.parse_args()
    main(args.current_rssi, args.rssi_rely)