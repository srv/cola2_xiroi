#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import Imu, MagneticField
from std_srvs.srv import Empty, EmptyResponse
import tf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import datetime
from numpy import *
import PyKDL
import sys
from std_srvs.srv import Empty
from std_srvs.srv import Trigger, TriggerRequest
from cola2_msgs.msg import GoalDescriptor, NavSts
from cola2_msgs.srv import Goto, GotoRequest
from cola2_xiroi.srv import SharePosition
from cola2_lib.utils.angles import wrap_angle
from cola2_lib.utils.ned import NED
from std_srvs.srv import Empty, EmptyRequest
from cola2_lib.rosutils import param_loader
from cola2_lib.rosutils.param_loader import get_ros_params


class ReturnToNED:
    

    def __init__(self, name):
        """ Init the class """
        # Get config parameters
        self.name = name
        self.ns = rospy.get_namespace()
        self.get_config()
        self.ned = NED(self.ned_origin_lat, self.ned_origin_lon, 0.0)

        # Services clients
        try:
            rospy.wait_for_service('captain/enable_goto', 20)
            self.goto_srv = rospy.ServiceProxy(
                        'captain/enable_goto', Goto)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to goto service',
                         self.name)
            rospy.signal_shutdown('Error creating client to goto service')

        try:
            rospy.wait_for_service('captain/disable_all_and_set_idle', 20)
            self.disable_all_and_set_idle_srv = rospy.ServiceProxy(
                        'captain/disable_all_and_set_idle', Trigger)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to disable_all service',
                         self.name)
            rospy.signal_shutdown('Error creating client to disable_all service')

        # Sevice server
        self.start_srv = rospy.Service('safety/return_to_NED',
                                       Empty,
                                       self.return_to_NED)

        # Show message
        rospy.loginfo('[%s]: initialized', self.name)
    

    def return_to_NED(self, req):
        """ This method sets the captain back to idle """
        try:
            self.disable_all_and_set_idle_srv(TriggerRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr("Error setting captain to idle state")

        """Goto to NED position."""
        # // Define waypoint attributes
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = 0.5
        goto_req.position.x = self.ned_origin_lat
        goto_req.position.y = self.ned_origin_lon
        goto_req.position.z = 0.0
        goto_req.position_tolerance.x = 2.0
        goto_req.position_tolerance.y = 2.0
        goto_req.position_tolerance.z = 2.0
        goto_req.blocking = True
        goto_req.keep_position = False
        goto_req.disable_axis.x = False
        goto_req.disable_axis.y = True
        goto_req.disable_axis.z = False
        goto_req.disable_axis.roll = True
        goto_req.disable_axis.yaw = False
        goto_req.disable_axis.pitch = True
        goto_req.priority = 60 #PRIORITY_TELEOPERATION_LOW = 0 PRIORITY_SAFETY_LOW = 5 PRIORITY_NORMAL = 10 PRIORITY_SAFETY = 30 PRIORITY_TELEOPERATION = 40 PRIORITY_SAFETY_HIGH  = 50 PRIORITY_TELEOPERATION_HIGH = 60
        goto_req.reference = 1 #REFERENCE_NED=0  REFERENCE_GLOBAL=1 REFERENCE_VEHICLE=2
        self.goto_srv(goto_req)
        rospy.sleep(1.0)


    def get_config(self):
        """Define and load all necessary parameters from ROS param server."""
        # Define params to load
        param_dict = {
            'ned_origin_lat': (self.ns + "navigator/ned_latitude", 41.7777),
            'ned_origin_lon': (self.ns + "navigator/ned_longitude", 3.0333),
            'max_distance': (self.ns + "captain/max_distance_to_waypoint", 400)}
        # Load them
        param_loader.get_ros_params(self, param_dict)
       

if __name__ == '__main__':
    try:
        rospy.init_node('return_to_NED')
        return_to_NED = ReturnToNED(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass