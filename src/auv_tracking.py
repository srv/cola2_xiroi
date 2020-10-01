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
from cola2_msgs.srv import Goto, GotoRequest,GotoResponse
from cola2_xiroi.srv import SharePosition
from cola2_lib.utils.angles import wrap_angle
from cola2_lib.utils.ned import NED
from std_srvs.srv import Empty, EmptyRequest
from cola2_lib.rosutils import param_loader
from cola2_lib.rosutils.param_loader import get_ros_params


class AUVTracking:
    

    def __init__(self, name):
        """ Init the class """
        # Initialize some parameters
        self.asv_data = False
        self.auv_data = False
        self.tracking = False
        self.max_distance = 10
        # Get config parameters
        self.name = name
        self.ns = rospy.get_namespace()
        # self.get_config()
        # self.ned = NED(self.ned_origin_lat, self.ned_origin_lon, 0.0)
        
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
                        '/xiroi/captain/disable_all_and_set_idle', Trigger)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to disable_all service',
                         self.name)
            rospy.signal_shutdown('Error creating client to disable_all service')
        
        
        # Sevice server
        self.enable_tracking_srv = rospy.Service('control/enable_tracking',
                                       Empty,
                                       self.enable_tracking)

        self.disable_tracking_srv = rospy.Service('control/disable_tracking',
                                       Empty,
                                       self.disable_tracking)
        
        #Subscribers
        #Xiroi position
        rospy.Subscriber(self.ns + "navigator/navigation",
                         NavSts,    
                         self.update_asv_nav_sts,
                         queue_size=1)
        #Turbot position
        rospy.Subscriber("/turbot/navigator/navigation",
                         NavSts,
                         self.update_auv_nav_sts,
                         queue_size=1)
   
    def update_asv_nav_sts(self, msg):
        self.asv_position_north = msg.position.north
        self.asv_position_east = msg.position.east
        self.asv_data = True

    def update_auv_nav_sts(self, msg):
        self.auv_position_north = msg.position.north
        self.auv_position_east = msg.position.east
        self.auv_data = True
        self.check_distance()

    def disable_tracking(self, req):
        self.tracking = False
        self.disable_all_and_set_idle_srv()
        rospy.logwarn('AUV tracking DISABLED')
        return EmptyResponse()

    def enable_tracking(self, req):
        self.tracking = True
        self.check_distance()
        return EmptyResponse()

    def start_tracking(self):
        """ This method sets the captain back to idle """
        self.disable_all_and_set_idle_srv()
        """Goto to AUV position."""
        # // Define Goto attributes    
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = 0.4
        goto_req.position.x = self.auv_position_north
        goto_req.position.y = self.auv_position_east
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
        goto_req.priority = 10 #PRIORITY_TELEOPERATION_LOW = 0 PRIORITY_SAFETY_LOW = 5 PRIORITY_NORMAL = 10 PRIORITY_SAFETY = 30 PRIORITY_TELEOPERATION = 40 PRIORITY_SAFETY_HIGH  = 50 PRIORITY_TELEOPERATION_HIGH = 60
        goto_req.reference = 0 #REFERENCE_NED=0  REFERENCE_GLOBAL=1 REFERENCE_VEHICLE=2
        self.goto_srv(goto_req)

    def check_distance(self):
        # print("check_distance")
        if(self.auv_data and self.asv_data and self.tracking) :
            self.distance = sqrt((self.auv_position_north-self.asv_position_north)**2 + (self.auv_position_east-self.asv_position_east)**2)
            print (self.distance)

        if(self.tracking and self.distance > self.max_distance):
            rospy.logwarn('AUV tracking ENABLED')
            self.start_tracking()

    def get_config(self):
        """Define and load all necessary parameters from ROS param server."""
        # Define params to load
        param_dict = {
            'ned_origin_lat': (self.ns + "navigator/ned_latitude", 41.7777),
            'ned_origin_lon': (self.ns + "navigator/ned_longitude", 3.0333)}
        # Load them
        param_loader.get_ros_params(self, param_dict)

if __name__ == '__main__':
    try:
        rospy.init_node('AUV_tracking')
        AUV_tracking = AUVTracking(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
