#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import re
import serial

import rospy
from sensor_msgs.msg import NavSatFix


def main():

    rospy.init_node(name='gps_node', anonymous=False)

    topic = rospy.get_param(param_name='~topic', default='/fix')
    pub = rospy.Publisher(topic, NavSatFix, queue_size=5)

    port = rospy.get_param(param_name='~port', default='/dev/ttyUSB0')
    baud = 9600
    ser = serial.Serial(port, baud)

    while True:
        while ser.in_waiting:
            try:
                data_raw = ser.readline()
                data = data_raw.decode()
                parse_data_and_pub(data=data, pub=pub)
            except Exception as err:
                print(err)
                rospy.loginfo(err)

def parse_data_and_pub(data, pub):

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
            # msg.altitude = 0.0
            msg.position_covariance = [3.706, 0, 0, 0, 3.706, 0, 0, 0, 3.706]  # 2.5 m CEP
            msg.position_covariance_type = 2
            pub.publish(msg)
        else:
            print("gps no signal")
            rospy.loginfo("gps no signal")
    else:
        pass

if __name__ == "__main__":

    main()
