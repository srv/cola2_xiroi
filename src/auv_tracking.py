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
import numpy as np 
from numpy import *
from math import *
import PyKDL
import sys
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger, TriggerRequest
from cola2_msgs.msg import GoalDescriptor, NavSts, BodyVelocityReq
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
        self.first_repulsion = False
        self.initialized = False
        self.radius= -1
        self.repulsion_radius = 10
        self.anchor_radius = self.repulsion_radius *3
        self.follow_radius = self.repulsion_radius*5
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
            rospy.wait_for_service('captain/enable_keep_position_holonomic', 20)
            self.enable_keep_position_holonomic_srv = rospy.ServiceProxy(
                        'captain/enable_keep_position_holonomic', Trigger)
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

        rospy.Subscriber("/turbot/controller/body_velocity_req",
                        BodyVelocityReq,
                        self.update_auv_bvr,
                        queue_size=1)
        #Publishers
        self.corrected_bvr = rospy.Publisher("/xiroi/controller/body_velocity_req",
                                                BodyVelocityReq,
                                                queue_size=1)
        self.markerPub_repulsion = rospy.Publisher('repulsionRadius',
                                                    Marker,
                                                    queue_size=1)

        self.markerPub_anchor = rospy.Publisher('anchorRadius',
                                                Marker,
                                                queue_size=1)

        self.markerPub_follow = rospy.Publisher('followRadius',
                                                Marker,
                                                queue_size=1)

        # Init periodic check timer
        rospy.Timer(rospy.Duration(0.1), self.check_distance)
   
    def update_auv_bvr(self, msg):
        self.velocity_request_linear_x = msg.twist.linear.x
        self.velocity_request_linear_y = msg.twist.linear.y
        self.velocity_request_linear_z = msg.twist.linear.z
        self.velocity_request_angular_x = msg.twist.angular.x
        self.velocity_request_angular_y = msg.twist.angular.y
        self.velocity_request_angular_z = msg.twist.angular.z
        self.disable_axis_x = msg.disable_axis.x
        self.disable_axis_y = msg.disable_axis.y
        self.disable_axis_z = msg.disable_axis.z
        self.disable_axis_roll= msg.disable_axis.roll
        self.disable_axis_pitch = msg.disable_axis.pitch
        self.disable_axis_yaw = msg.disable_axis.yaw

    def update_asv_nav_sts(self, msg):
        self.asv_position_north = msg.position.north
        self.asv_position_east = msg.position.east
        self.asv_yaw = msg.orientation.yaw
        self.asv_data = True

    def update_auv_nav_sts(self, msg):
        self.auv_position_north = msg.position.north
        self.auv_position_east = msg.position.east
        self.auv_position_depth = msg.position.depth
        self.anchor_marker()
        self.repulsion_marker()
        self.follow_marker()
        self.auv_data = True

    def disable_tracking(self, req):
        self.disable_goto_srv()
        self.disable_keep_position_srv()
        self.tracking = False
        return EmptyResponse()

    def enable_tracking(self, req):
        self.tracking = True
        return EmptyResponse()


    def start_tracking(self, position_x, position_y):
        self.disable_goto_srv()
        """ This method sets the captain back to idle """
        """Goto to AUV position."""
        # // Define Goto attributes    
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = 5.0
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
        goto_req.priority = 0 #PRIORITY_TELEOPERATION_LOW = 0 PRIORITY_SAFETY_LOW = 5 PRIORITY_NORMAL = 10 PRIORITY_SAFETY = 30 PRIORITY_TELEOPERATION = 40 PRIORITY_SAFETY_HIGH  = 50 PRIORITY_TELEOPERATION_HIGH = 60
        goto_req.reference = 0 #REFERENCE_NED=0  REFERENCE_GLOBAL=1 REFERENCE_VEHICLE=2
        self.goto_srv(goto_req)

    def check_distance(self,event):
        if(self.auv_data and self.asv_data and self.tracking) :
            self.x_distance = self.auv_position_north-self.asv_position_north
            self.y_distance = self.auv_position_east-self.asv_position_east
            self.radius = sqrt((self.x_distance)**2 + (self.y_distance)**2)
            self.initialized = True
            print (self.radius)
            

        if(self.radius > self.anchor_radius):
            print("FOLLOW")
            self.first_repulsion = True
            # self.enable_thrusters_srv()
            self.disable_keep_position_srv() 
            # self.start_tracking(self.auv_position_north,self.auv_position_east)
            self.follow_strategy()

        elif(self.repulsion_radius <= self.radius <= self.anchor_radius):
            self.first_repulsion = True
            # self.disable_goto_srv()
            print("WAIT")
            self.enable_keep_position_holonomic_srv() 

        elif(self.radius <= self.repulsion_radius and self.initialized):
            print("REPULSION")
            self.disable_keep_position_srv()
            # if (self.first_repulsion):
            self.extract_safety_position()
            self.repulsion_strategy(-self.x, -self.y)
            # self.first_repulsion = False 

    def extract_safety_position(self):
        print("EXTRACT SAFETY POSITION")
        self.m = (self.auv_position_east-self.asv_position_east)/(self.auv_position_north-self.asv_position_north)
        # self.x =  (self.auv_position_north - self.asv_position_north) + self.repulsion_radius
        self.x =  (self.auv_position_north - self.asv_position_north) 
        self.y = (self.m*(self.x-self.asv_position_north))+ self.asv_position_east
        return

    def repulsion_strategy(self, position_x, position_y):
        print("REPULSION STRATEGY")
        constant_linear_velocity = 0.9
        constant_angular_velocity = 25 
        linear_velocity = constant_linear_velocity
        alpha_ref = atan2(position_y,position_x)
        #obtain the minimum agle between both robots
        angle_error = alpha_ref - self.asv_yaw
        # angle_error = atan2(sin(alpha_ref-self.asv_yaw), cos(alpha_ref-self.asv_yaw))
        self.angular_velocity = constant_angular_velocity * angle_error
        self.xr = linear_velocity*cos(angle_error)
        self.yr = linear_velocity*sin(angle_error)


        self.corrected_bvr_pusblisher(self.xr, self.yr,self.angular_velocity)


    def follow_strategy(self):
        constant_linear_velocity = 7
        constant_angular_velocity = 25 

        if (self.radius<self.follow_radius and self.radius>self.anchor_radius):
            # Normalice the radius
            self.velocity_adjustment = (self.radius-(self.anchor_radius))/(self.follow_radius-(self.anchor_radius))
        else:
            self.velocity_adjustment = 1  

        print(self.velocity_adjustment)
        linear_velocity = self.velocity_adjustment * constant_linear_velocity
        alpha_ref = atan2(self.y_distance,self.x_distance)
        #obtain the minimum agle between both robots
        angle_error = atan2(sin(alpha_ref-self.asv_yaw), cos(alpha_ref-self.asv_yaw))

        self.angular_velocity = constant_angular_velocity * angle_error
        self.xr = linear_velocity*cos(angle_error)
        self.yr = linear_velocity*sin(angle_error)

        if(self.xr < 0 ):
            self.xr = 0

        self.corrected_bvr_pusblisher(self.xr, self.yr,self.angular_velocity)
    
    def corrected_bvr_pusblisher(self,corrected_velocity_x,corrected_velocity_y,corrected_angular_z):
        # NEW PRIORITY DEFINITIONS
        # PRIORITY_TELEOPERATION_LOW = 0
        # PRIORITY_SAFETY_LOW = 5
        # PRIORITY_NORMAL = 10
        # PRIORITY_NORMAL_HIGH = 20
        # PRIORITY_TELEOPERATION = 40
        # PRIORITY_SAFETY = 45
        # PRIORITY_SAFETY_HIGH  = 50
        # PRIORITY_TELEOPERATION_HIGH = 60 
        print ("bvr publisher")
        bvr = BodyVelocityReq()
        bvr.header.frame_id    = "/xiroi/base_link"
        bvr.header.stamp       = rospy.Time.now()
        bvr.goal.requester     =  self.name
        bvr.disable_axis.x     = False
        bvr.disable_axis.y     = False
        bvr.disable_axis.z     = True
        bvr.disable_axis.roll  = True
        bvr.disable_axis.pitch = True
        bvr.disable_axis.yaw   = False
        bvr.twist.linear.x     = corrected_velocity_x
        bvr.twist.linear.y     = corrected_velocity_y
        bvr.twist.linear.z     = 0.0
        bvr.twist.angular.x    = 0.0
        bvr.twist.angular.y    = 0.0
        bvr.twist.angular.z    = corrected_angular_z
        bvr.goal.priority      = 20
        self.corrected_bvr.publish(bvr)

    def anchor_marker(self):
        self.anchor = Marker()
        self.anchor.header.frame_id = "world_ned"
        self.anchor.header.stamp = rospy.Time.now()
        self.anchor.ns = "turbot"
        self.anchor.id = 0
        self.anchor.type = Marker.CYLINDER
        self.anchor.action = Marker.ADD
        self.anchor.pose.position.x = self.auv_position_north
        self.anchor.pose.position.y = self.auv_position_east
        self.anchor.pose.position.z = 0
        self.anchor.pose.orientation.x = 0
        self.anchor.pose.orientation.y = 0
        self.anchor.pose.orientation.z = 0
        self.anchor.pose.orientation.w = 1.0
        self.anchor.scale.x = self.anchor_radius*2
        self.anchor.scale.y = self.anchor_radius*2
        self.anchor.scale.z = 0.1
        self.anchor.color.r = 0.0
        self.anchor.color.g = 1.0
        self.anchor.color.b = 0.0
        self.anchor.color.a = 0.3
        self.markerPub_anchor.publish(self.anchor)

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
        self.follow.scale.x = self.follow_radius*2
        self.follow.scale.y = self.follow_radius*2
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
