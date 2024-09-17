#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
import math
from std_msgs.msg import Header, Float64
from rtk_driver.msg import rtk_msg

#read the serial port
serial_port = rospy.get_param('~port','/dev/ttyUSB0')
serial_baud = rospy.get_param('~baudrate',57600)
serialread = serial.Serial(serial_port, serial_baud,timeout=3.)

#function:convert the gngga format into decimal format(for latitude and longitude)

def convert_to_decimal(ddmm_format):
    degree = math.trunc(ddmm_format/100)
    minute = math.trunc((ddmm_format/100-degree)*100)/60
    second = (ddmm_format-math.trunc(ddmm_format))*100/3600
    decimal_data = degree + minute + second
    return decimal_data

#publishing the msg            

if __name__ == '__main__':
    try:
#create the node

        pub_topic = rospy.Publisher('rtk', rtk_msg, queue_size=10)
        rospy.init_node('rtk_data_publisher')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

#read the gpgga data 

            gpgga_data = serialread.readline().decode('utf-8')
            gpgga_data_split = gpgga_data.split(',')
            gpgga_head = gpgga_data_split[0]
            if gpgga_head == '/r$GNGGA':
                if gpgga_data_split[2] != '':
                    latitude = float(gpgga_data_split[2])
                    latitude_direction = gpgga_data_split[3]
                    longitude = float(gpgga_data_split[4])
                    longitude_direction = gpgga_data_split[5]
                    gps_fix = gpgga_data_split[6]
                    altitude = float(gpgga_data_split[9])

    #convert the raw data of longitude and latitude to decimal data

                    latitude_decimal = convert_to_decimal(latitude)
                    longitude_decimal = convert_to_decimal(longitude)

    #check the direction of latitude and longitude  

                    if latitude_direction == 'S':
                        latitude_decimal = -latitude_decimal
                    if longitude_direction == 'W':
                        longitude_decimal = -longitude_decimal

    #convert it to utm format

                    utm_easting, utm_northing, zone, letter = utm.from_latlon(latitude_decimal, longitude_decimal)

    #transfer the variables to msg so that it will be published to the rostopic /gps

                    msg = rtk_msg()
                    msg.header = Header()
                    msg.header.stamp.secs = math.trunc(float(gpgga_data_split[1]))
                    msg.header.frame_id = 'GPS2_Frame'
                    msg.latitude = latitude_decimal
                    msg.longitude = longitude_decimal
                    msg.altitude = altitude
                    msg.gps_fix = gps_fix
                    msg.utm_easting = utm_easting
                    msg.utm_northing = utm_northing
                    msg.zone = zone
                    msg.letter = letter

                    rospy.loginfo(msg)
                    pub_topic.publish(msg)
                    
                    rate.sleep()
                    
    except rospy.ROSInterruptException:
        pass
