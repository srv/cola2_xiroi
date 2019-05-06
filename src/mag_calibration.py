#!/usr/bin/env python


import roslib
import rospy

from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu

import datetime
from numpy import *



class Mag_Calibration:
    def __init__(self, name):
        """ Init the class """
        self.name = name

        # Get calibration params
        self.soft_iron = rospy.get_param("sensors/imu/soft_iron", default=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.hard_iron = rospy.get_param("sensors/imu/hard_iron",default=[[0], [0], [0]])

        # Subscriber
        rospy.Subscriber('mag_driver', MagneticField, self.magCb, queue_size=1)
        rospy.Subscriber('imu_driver', Imu, self.imuCb, queue_size=1)

        # Publisher
        self.pub_mag = rospy.Publisher('mag', MagneticField, queue_size = 2)
        self.pub_imu = rospy.Publisher('imu', Imu, queue_size = 2)

        # ADD services 

    def imuCb(self, msg):
        acc_input = matrix([[msg.linear_acceleration.x],
                            [msg.linear_acceleration.y],
                            [msg.linear_acceleration.z]])


        # MAGNETOMETER DRIVER CORRECTION
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        msg.linear_acceleration.x = -ay
        msg.linear_acceleration.y = ax
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = -wy
        msg.angular_velocity.y = wx
        msg.angular_velocity.z = wz

        self.pub_imu.publish(msg)

    def magCb(self, msg):
        mag_input = matrix([[msg.magnetic_field.x],
                            [msg.magnetic_field.y],
                            [msg.magnetic_field.z]])
        #mag_calib =  self.soft_iron * (mag_input - self.hard_iron)  
        mag_calib =  mag_input + self.hard_iron

        # NWU to ENU
        # msg.magnetic_field.x = - mag_calib[1]
        # msg.magnetic_field.y = mag_calib[0]
        # msg.magnetic_field.z = mag_calib[2]

        # MAGNETOMETER DRIVER CORRECTION
        x = mag_calib[0]
        y = -mag_calib[1]
        z = -mag_calib[2]

        msg.magnetic_field.x = y
        msg.magnetic_field.y = x
        msg.magnetic_field.z = -z

        msg.magnetic_field.x = x
        msg.magnetic_field.y = y
        msg.magnetic_field.z = z


        msg.magnetic_field_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        self.pub_mag.publish(msg)

    def startCalibration(self): # SERVICE Store MAG measurements
        return

    def computeCalibration(self): # SERVICE Compute calibration
        return

    def removeData(self): # SERVICE Remove MAG data
        return


if __name__ == '__main__':
    try:
        rospy.init_node('mag_calibration')
        angle_estimator = Mag_Calibration(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
