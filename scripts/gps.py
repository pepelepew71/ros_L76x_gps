#!/usr/bin/env python

from __future__ import print_function

import re
import serial

import rospy
from sensor_msgs.msg import NavSatFix


def main():

    port = rospy.get_param(param_name='~port', default='/dev/ttyUSB0')
    topic = rospy.get_param(param_name='~topic', default='/fix')

    rospy.init_node(name='L76X_gps', anonymous=False)
    pub = rospy.Publisher(topic, NavSatFix, queue_size=5)

    baud = 9600
    ser = serial.Serial(port, baud)

    while True:
        while ser.in_waiting:

            data_raw = ser.readline()
            data = data_raw.decode()

            if data[:6] == "$GNGLL":

                data_list = data.split(",")
                lat = data_list[1]
                lon = data_list[3]

                if lat and lon:

                    lat = float(lat[:-7]) + float(lat[-7:]) / 60.0  # google map DD format
                    lon = float(lon[:-7]) + float(lon[-7:]) / 60.0

                    msg = NavSatFix()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "/world"
                    msg.latitude = lat
                    msg.longitude = lon
                    msg.altitude = 0.0
                    msg.position_covariance = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
                    msg.position_covariance_type = 0

                    pub.publish(msg)

                else:
                    print("gps no signal")
                    rospy.info("gps no signal")

if __name__ == "__main__":

    main()
