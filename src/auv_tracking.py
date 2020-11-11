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
from visualization_msgs.msg import Marker
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
        self.initialized = False
        self.radius=11
        self.acceptance_radius = 50 
        self.goal_radius = self.acceptance_radius - 5 
        self.repulsion_radius = 40
        self.depth_threshold = 3
        
        # Get config parameters
        self.name = name
        self.ns = rospy.get_namespace()
        
        # Services clients
        #enable_goto service
        try:
            rospy.wait_for_service('captain/enable_goto', 20)
            self.goto_srv = rospy.ServiceProxy(
                        'captain/enable_goto', Goto)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to goto service',
                         self.name)
            rospy.signal_shutdown('Error creating client to goto service')
        #disable_goto service
        try:
            rospy.wait_for_service('captain/disable_goto', 20)
            self.disable_goto_srv = rospy.ServiceProxy(
                        'captain/disable_goto', Trigger)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to disable goto service',
                         self.name)
            rospy.signal_shutdown('Error creating client to disable goto service')
        # enable_keep_position service
        try:
            rospy.wait_for_service('captain/enable_keep_position_non_holonomic', 20)
            self.enable_keep_position_non_holonomic_srv = rospy.ServiceProxy(
                        'captain/enable_keep_position_non_holonomic', Trigger)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to captain/enable_keep_position_non_holonomic service',
                         self.name)
            rospy.signal_shutdown('Error creating client to captain/enable_keep_position_non_holonomic service')
        # disable_keep_position service
        try:
            rospy.wait_for_service('captain/disable_keep_position', 20)
            self.disable_keep_position_srv = rospy.ServiceProxy(
                        'captain/disable_keep_position', Trigger)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to captain /disable_keep_position service',
                         self.name)
            rospy.signal_shutdown('Error creating client to captain /disable_keep_position service')

        # enable thrusters service
        rospy.wait_for_service('controller/enable_thrusters', 10)
        try:
            self.enable_thrusters_srv = rospy.ServiceProxy(
                        'controller/enable_thrusters', Empty)
        except rospy.ServiceException, e:
            rospy.logwarn("%s: Service call failed: %s", self.name, e)

        # disable thrusters service
        rospy.wait_for_service('controller/disable_thrusters', 10)
        try:
            self.disable_thrusters_srv = rospy.ServiceProxy(
                        'controller/disable_thrusters', Empty)
        except rospy.ServiceException, e:
            rospy.logwarn("%s: Service call failed: %s", self.name, e)
        #set IDLE
        try:
            rospy.wait_for_service('captain/disable_all_and_set_idle', 20)
            self.disable_all_and_set_idle_srv = rospy.ServiceProxy(
                        'captain/disable_all_and_set_idle', Trigger)
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
        rospy.Subscriber(self.ns + "navigator/navigation",
                         NavSts,    
                         self.update_asv_nav_sts,
                         queue_size=1)

        rospy.Subscriber("/turbot/navigator/navigation",
                         NavSts,
                         self.update_auv_nav_sts,
                         queue_size=1)
        #Publishers
        self.markerPub_repulsion = rospy.Publisher('repulsionRadius',Marker,queue_size=1)
        self.markerPub_goal = rospy.Publisher('goalRadius',Marker,queue_size=1)
        self.markerPub_follow = rospy.Publisher('followRadius',Marker,queue_size=1)

        # Init periodic check timer
        rospy.Timer(rospy.Duration(1), self.check_distance)
   
    def update_asv_nav_sts(self, msg):
        self.asv_position_north = msg.position.north
        self.asv_position_east = msg.position.east
        self.asv_data = True

    def update_auv_nav_sts(self, msg):
        self.auv_position_north = msg.position.north
        self.auv_position_east = msg.position.east
        self.goal_marker()
        self.repulsion_marker()
        self.follow_marker()
        self.auv_data = True
        self.disable_goto_srv()

    def disable_tracking(self, req):
        self.disable_goto_srv()
        self.disable_keep_position_srv()
        self.tracking = False
        return EmptyResponse()

    def enable_tracking(self, req):
        self.tracking = True
        return EmptyResponse()


    def start_tracking(self, position_x, position_y):
        """ This method sets the captain back to idle """
        """Goto to AUV position."""
        # // Define Goto attributes    
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = 2.0
        goto_req.position.x = position_x
        goto_req.position.y = position_y
        goto_req.position.z = 0.0
        goto_req.position_tolerance.x = 2.0
        goto_req.position_tolerance.y = 2.0
        goto_req.position_tolerance.z = 2.0
        goto_req.blocking = False #True
        goto_req.keep_position = False
        goto_req.disable_axis.x = False
        goto_req.disable_axis.y = True
        goto_req.disable_axis.z = False
        goto_req.disable_axis.roll = True
        goto_req.disable_axis.yaw = False
        goto_req.disable_axis.pitch = True
        goto_req.priority = 5 #PRIORITY_TELEOPERATION_LOW = 0 PRIORITY_SAFETY_LOW = 5 PRIORITY_NORMAL = 10 PRIORITY_SAFETY = 30 PRIORITY_TELEOPERATION = 40 PRIORITY_SAFETY_HIGH  = 50 PRIORITY_TELEOPERATION_HIGH = 60
        goto_req.reference = 0 #REFERENCE_NED=0  REFERENCE_GLOBAL=1 REFERENCE_VEHICLE=2
        self.goto_srv(goto_req)
        rospy.sleep(1.0)

    def check_distance(self,event):
        if(self.auv_data and self.asv_data and self.tracking) :
            self.x_distance = self.auv_position_north-self.asv_position_north
            self.y_distance = self.auv_position_east-self.asv_position_east
            self.radius = sqrt((self.x_distance)**2 + (self.y_distance)**2)
            print (self.radius)
            self.initialized = True

        if(self.radius > self.goal_radius):
            self.disable_keep_position_srv()
            self.start_tracking(self.auv_position_north,self.auv_position_east)

        elif(self.repulsion_radius <= self.radius <= self.goal_radius):
            self.enable_keep_position_non_holonomic_srv()

        elif(self.radius <= self.repulsion_radius and self.initialized):
            self.disable_keep_position_srv()
            self.extract_safety_position()
            self.start_tracking(self.x,self.y)

    def extract_safety_position(self):
        self.m = (self.auv_position_east-self.asv_position_east)/(self.auv_position_north-self.asv_position_north)
        self.x =  (self.auv_position_north - self.asv_position_north) + self.acceptance_radius
        self.y = (self.m*(self.x-self.asv_position_north))+self.asv_position_east

    def goal_marker(self):
        self.goal = Marker()
        self.goal.header.frame_id = "world_ned"
        self.goal.header.stamp = rospy.Time.now()
        self.goal.ns = "turbot"
        self.goal.id = 0
        self.goal.type = Marker.CYLINDER
        self.goal.action = Marker.ADD
        self.goal.pose.position.x = self.auv_position_north
        self.goal.pose.position.y = self.auv_position_east
        self.goal.pose.position.z = 0
        self.goal.pose.orientation.x = 0
        self.goal.pose.orientation.y = 0
        self.goal.pose.orientation.z = 0
        self.goal.pose.orientation.w = 1.0
        self.goal.scale.x = self.goal_radius*2
        self.goal.scale.y = self.goal_radius*2
        self.goal.scale.z = 0.1
        self.goal.color.r = 0.0
        self.goal.color.g = 1.0
        self.goal.color.b = 0.0
        self.goal.color.a = 0.3
        self.markerPub_goal.publish(self.goal)

    def repulsion_marker(self):
        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = "world_ned"
        self.robotMarker.header.stamp = rospy.Time.now()
        self.robotMarker.ns = "turbot"
        self.robotMarker.id = 0
        self.robotMarker.type = Marker.CYLINDER
        self.robotMarker.action = Marker.ADD
        self.robotMarker.pose.position.x = self.auv_position_north
        self.robotMarker.pose.position.y = self.auv_position_east
        self.robotMarker.pose.position.z = 0
        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = self.repulsion_radius*2
        self.robotMarker.scale.y = self.repulsion_radius*2
        self.robotMarker.scale.z = 0.1
        self.robotMarker.color.r = 1.0
        self.robotMarker.color.g = 0.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 0.3
        self.markerPub_repulsion.publish(self.robotMarker)

    def follow_marker(self):
        self.follow = Marker()
        self.follow.header.frame_id = "world_ned"
        self.follow.header.stamp = rospy.Time.now()
        self.follow.ns = "turbot"
        self.follow.id = 0
        self.follow.type = Marker.CYLINDER
        self.follow.action = Marker.ADD
        self.follow.pose.position.x = self.auv_position_north
        self.follow.pose.position.y = self.auv_position_east
        self.follow.pose.position.z = 0
        self.follow.pose.orientation.x = 0
        self.follow.pose.orientation.y = 0
        self.follow.pose.orientation.z = 0
        self.follow.pose.orientation.w = 1.0
        self.follow.scale.x = self.acceptance_radius*2
        self.follow.scale.y = self.acceptance_radius*2
        self.follow.scale.z = 0.1
        self.follow.color.r = 0.0
        self.follow.color.g = 0.0
        self.follow.color.b = 1.0
        self.follow.color.a = 0.3
        self.markerPub_follow.publish(self.follow)

if __name__ == '__main__':
    try:
        rospy.init_node('AUV_tracking')
        AUV_tracking = AUVTracking(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
