#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.



import rospy

from cola2_msgs.msg import Setpoints
from sensor_msgs.msg import Joy

from math import *
import numpy as np


class TestActuators485:
    """ Just to test the actuators_485.cpp node """

    def __init__(self, name):
        """ Constructor """
        self.name = name

        # Publisher
        self.pub_setpoints = rospy.Publisher("cola2_control/thrusters_data", 
                                             Setpoints,
                                             queue_size = 2)

        # Subscriber
        rospy.Subscriber("/joy", Joy, self.update_joy)

        # Timer for check_sensors method
        rospy.Timer(rospy.Duration(0.1), self.timer_callback) # Always < 0.2

        # Message
        self.msg = Setpoints()
        self.msg.setpoints = np.array([0.0, 0.0, 0.0])


    def update_joy(self, data):
        aux = np.array([0.0, 0.0, 0.0])
        aux[0] = -data.axes[1] * abs(data.axes[1])
        aux[1] = data.axes[4] * abs(data.axes[4]) + data.axes[0] * abs(data.axes[0])
        aux[2] = data.axes[4] * abs(data.axes[4]) - data.axes[0] * abs(data.axes[0])
        self.msg.setpoints = aux.clip(min=-1.0, max=1.0)


    def timer_callback(self, event):
        """ Callback of the timer """
        self.pub_setpoints.publish(self.msg)


if __name__ == '__main__':
    try:
        # Init node
        rospy.init_node('test_actuators_485')
        test_actuators_485 = TestActuators485(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
