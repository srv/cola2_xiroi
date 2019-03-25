#!/usr/bin/env python


import roslib
import rospy

from sensor_msgs.msg import MagneticField

import datetime
from numpy import *



class Mag_Calibration:
    def __init__(self, name):
        """ Init the class """
        self.name = name

        # Get calibration params
        self.soft_iron = rospy.get_param("sensors/imu/soft_iron")
        self.hard_iron = rospy.get_param("sensors/imu/hard_iron")

        # Subscriber
        rospy.Subscriber('mag_driver', MagneticField, self.magCb, queue_size=1)

        # Publisher
        self.pub = rospy.Publisher('mag', MagneticField, queue_size = 2)



    def magCb(self, msg):
        mag_input = matrix([[msg.magnetic_field.x],
                            [msg.magnetic_field.y],
                            [msg.magnetic_field.z]])
        mag_calib =  self.soft_iron * (mag_input - self.hard_iron)  

        msg.magnetic_field.x = mag_calib[0]
        msg.magnetic_field.y = mag_calib[1]
        msg.magnetic_field.z = mag_calib[2]
        msg.magnetic_field_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('mag_calibration')
        angle_estimator = Mag_Calibration(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
