#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.



"""
@@>This node is used to simulate Sparus II actuators. It is only used in simulation.<@@"""
"""
Modified 11/2015
"""
# Basic ROS imports
import rospy

# Import msgs
from cola2_msgs.msg import Setpoints

# More imports
import numpy as np


class SimActuatorsXi :
    """ Simulates thrusters and fins of Sparus2 AUV, converting from setpoints
        to angles or rpm """

    def __init__(self, name):
        """ Constructor """
        self.name = name
        self.last_trusters_update = rospy.Time().now()

        # Create publishers
        self.pub_thrusters = rospy.Publisher(
                                   '/cola2_control/sim_thrusters_data',
                                   Setpoints,
                                   queue_size=2)


        # Create subscribers
        rospy.Subscriber('/cola2_control/thrusters_data',
                         Setpoints,
                         self.update_thrusters,
                         queue_size=1)

        # Timer
        rospy.Timer(rospy.Duration(1.0), self.check_thrusters)

        # Show message
        rospy.loginfo("%s: initialized", self.name)


    def update_thrusters(self, thrusters) :
        """ Thrusters callback """
        self.last_trusters_update = rospy.Time().now()
        msg = Setpoints()
        msg.header = thrusters.header

        # Linear approach with max rpm of 1200
        msg.setpoints = 1200 * np.array(thrusters.setpoints).clip(min=-1,
                                                                  max=1)
        self.pub_thrusters.publish(msg)


    def check_thrusters(self, event):
        """ Method used to tell dynamics node that thrusters stopped due to
            not receiving data """
        if (rospy.Time().now() - self.last_trusters_update).to_sec() > 2.0:
            msg = Setpoints()
            msg.setpoints = np.zeros(3)  # Number of thrusters here
            self.pub_thrusters.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('sim_actuators_xi')
        sim_actuators_xi = SimActuatorsXi(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
