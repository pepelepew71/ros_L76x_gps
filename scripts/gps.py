#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import os
import re
import serial

import numpy as np

import rospy
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import SetBool, SetBoolResponse

def cb_save(request):
    path = os.getcwd() + "/gps_records.csv"
    rospy.loginfo("/gps: save to " + path)
    np.savetxt(path, np.array(RECORDS), delimiter=",")
    return EmptyResponse()

def cb_start(request):
    global RECORDS, IS_RECORD

    if request.data:
        rospy.loginfo("/gps: start recording")
        IS_RECORD = True
    else:
        rospy.loginfo("/gps: stop recording and delete all")
        RECORDS = list()
        IS_RECORD = False

def parse_data_and_pub(data, pub):
    global RECORDS

    if data[:6] == "$GNGLL":
        data_list = data.split(",")
        lat = data_list[1]
        lon = data_list[3]
        if lat and lon:
            lat = float(lat[:-8]) + float(lat[-8:]) / 60.0  # google map DD format
            lon = float(lon[:-8]) + float(lon[-8:]) / 60.0
            if IS_RECORD:
                RECORDS.append((lat, lon))
            msg = NavSatFix()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "/world"
            msg.latitude = lat
            msg.longitude = lon
            msg.position_covariance = [3.706, 0, 0, 0, 3.706, 0, 0, 0, 3.706]  # 2.5 m CEP
            msg.position_covariance_type = 2
            pub.publish(msg)
        else:
            print("gps no signal")
            rospy.loginfo("gps no signal")
    else:
        pass

if __name__ == "__main__":

    rospy.init_node(name='gps_node', anonymous=False)

    topic = rospy.get_param(param_name='~topic', default='/gps_fix')
    port = rospy.get_param(param_name='~port', default='/dev/ttyUSB0')
    baud = rospy.get_param(param_name='~baud', default=9600)

    rospy.Service(name='~save', service_class=Empty, handler=cb_save)
    rospy.Service(name='~start', service_class=SetBool, handler=cb_start)

    pub = rospy.Publisher(topic, NavSatFix, queue_size=5)
    ser = serial.Serial(port, int(baud))

    IS_RECORD = False
    RECORDS = []

    while True:
        while ser.in_waiting:
            try:
                data_raw = ser.readline()
                data = data_raw.decode()
                parse_data_and_pub(data=data, pub=pub)
            except Exception as err:
                print(err)
                rospy.loginfo(err)
