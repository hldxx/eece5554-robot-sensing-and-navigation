#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import math
from std_msgs.msg import Header
import numpy as np
from imu_driver.msg import imu_msg

#read the serial port
serial_port = rospy.get_param('/imu_data_publisher/port')
serial_baud = rospy.get_param('~baudrate',115200)
serialread = serial.Serial(serial_port, serial_baud)

def euler_to_quaternion(roll,pitch,yaw):
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    quaternionw = (np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)) + (np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
    quaternionx = (np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)) - (np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
    quaterniony = (np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)) + (np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2))
    quaternionz = (np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)) - (np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2))
    return quaternionw, quaternionx, quaterniony, quaternionz



#publishing the msg            

if __name__ == '__main__':
    try:
#create the node

        rospy.init_node('imu_data_publisher')
        pub_topic = rospy.Publisher('imu', imu_msg, queue_size=10)
        rospy.logdebug('reading the data')
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():

#read the imu data 

            imu_data = serialread.readline().decode('utf-8')
            
            #decoded_imu_data = imu_data.decode('utf-8')
            imu_data_split = imu_data.split(',')
            imu_head = imu_data_split[0]
            if '$VNYMR' not in imu_head:
                rospy.logwarn('data type is not correct')
            elif imu_data_split[2]=='':
                rospy.logwarn('data is empty')
            elif len(imu_data_split)!=13:
                rospy.logwarn('data is wrong')
            else:
                roll = float(imu_data_split[3])
                pitch = float(imu_data_split[2])
                yaw = float(imu_data_split[1])

                msg = imu_msg()
                msg.Header = Header()
                msg.Header.stamp = rospy.Time.now()
                msg.IMU.header.stamp = rospy.Time.now()
                msg.MagField.header.stamp = rospy.Time.now()
                msg.Header.frame_id = 'IMU1_Frame'
                msg.IMU.header.frame_id = 'IMU1_Frame'
                msg.MagField.header.frame_id = 'IMU1_Frame'

                msg.IMU.orientation.x,msg.IMU.orientation.y,msg.IMU.orientation.z,msg.IMU.orientation.w = euler_to_quaternion(roll,pitch,yaw)
                msg.MagField.magnetic_field.x = float(imu_data_split[4])
                msg.MagField.magnetic_field.y = float(imu_data_split[5])
                msg.MagField.magnetic_field.z = float(imu_data_split[6])
                msg.IMU.linear_acceleration.x = float(imu_data_split[7])
                msg.IMU.linear_acceleration.y = float(imu_data_split[8])
                msg.IMU.linear_acceleration.z = float(imu_data_split[9])
                msg.IMU.angular_velocity.x = float(imu_data_split[10])
                msg.IMU.angular_velocity.y = float(imu_data_split[11])
                msg.IMU.angular_velocity.z = float(imu_data_split[12].split('*')[0])
                msg.rawIMUstring = imu_data

                rospy.loginfo(msg)
                pub_topic.publish(msg)
                rate.sleep()

                
                    
    except rospy.ROSInterruptException:
        pass
