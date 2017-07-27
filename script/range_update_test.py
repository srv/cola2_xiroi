#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


# Basic ROS imports
import rospy

# Import msgs
from cola2_msgs.msg import RangeDetection
from auv_msgs.msg import NavSts
from cola2_msgs.srv import AddLandmark, AddLandmarkRequest

# More imports
import numpy as np
import math
import random

class RangeUpdateTest :
    """ Simulates thrusters and fins of Sparus2 AUV, converting from setpoints
        to angles or rpm """

    def __init__(self, name):
        """ Constructor """
        self.name = name
        self.last_nav = NavSts()
        self.landmark = [0.0, 0.0, 0.0]
        # Add landmark at x, y, z
        try:
            rospy.wait_for_service('/cola2_navigation/add_landmark', 20)
            self.add_landmark_client = rospy.ServiceProxy(
                        '/cola2_navigation/add_landmark', AddLandmark)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s, Error creating client to add landmark.',
                         self.name)
            rospy.signal_shutdown('Error creating add landmark client')
        req = AddLandmarkRequest();
        req.id.data = "docking";
        sigma = 0.000001
        req.landmark.pose.position.x = self.landmark[0]
        req.landmark.pose.position.y = self.landmark[1]
        req.landmark.pose.position.z = self.landmark[2]
        req.landmark.covariance[0] = sigma
        req.landmark.covariance[7] = sigma
        req.landmark.covariance[14] = sigma
        req.landmark.covariance[21] = sigma
        req.landmark.covariance[28] = sigma
        req.landmark.covariance[35] = sigma
        self.add_landmark_client(req)

        # Create publishers
        self.pub_range = rospy.Publisher(
                                   '/cola2_navigation/range_update',
                                   RangeDetection,
                                   queue_size = 2)

        # Subscriber
        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.update_nav,
                         queue_size = 1)

    def update_nav(self, nav):
        self.last_nav = nav

    def iteration(self):
        range = RangeDetection()
        range.header.stamp = rospy.Time.now()
        range.header.frame_id = '/girona500'
        range.id = 'docking'
        range.range.data = math.sqrt((self.last_nav.position.north - self.landmark[0])**2 +
                                     (self.last_nav.position.east - self.landmark[1])**2 +
                                     (self.last_nav.position.depth - self.landmark[2])**2) + (random.random()*4.0 - 2.0)
        range.sigma.data = 1.0

        self.pub_range.publish(range)

if __name__ == '__main__':
    try:
        rospy.init_node('range_update_test')
        RUT = RangeUpdateTest(rospy.get_name())

        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            RUT.iteration()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
