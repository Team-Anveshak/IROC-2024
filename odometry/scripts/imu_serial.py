#!/usr/bin/env python3

import time
import math
import rospy
import serial
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler


serial_port = '/dev/ttyUSB0'
baud_rate = 115200
imu_msg = Imu()

imu = serial.Serial(port=serial_port,baudrate=baud_rate, timeout=0.1)
imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)

if __name__ == "__main__":

    rospy.init_node("Imu_KalmanFilter", anonymous=False)
    try:
        while True:

            data = imu.readline().decode()
            imu_values = data.split(',')
            
            RollAngle = float(imu_values[1][7:])
            PitchAngle = float(imu_values[0][6:])
            YawAngle = float(imu_values[2][5:])

            imu_quaternions = quaternion_from_euler(RollAngle, PitchAngle, YawAngle)

            imu_msg.header.frame_id = "imu_link"
            imu_msg.orientation.x = imu_quaternions[0]
            imu_msg.orientation.y = imu_quaternions[1]
            imu_msg.orientation.z = imu_quaternions[2]
            imu_msg.orientation.w = imu_quaternions[3]
            imu_pub.publish(imu_msg)

            # print("Roll :", RollAngle, "Pitch :",PitchAngle, "Yaw :", YawAngle)
            # print(imu_quaternions)
            
            time.sleep(0.1)
        
    except KeyboardInterrupt:

        imu.close()