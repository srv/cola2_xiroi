#!/usr/bin/env python
"""@@This node is used to simulate navigation sensors. It is only used
in simulation.@@"""

# Basic ROS imports
import roslib
roslib.load_manifest('cola2_xiroi')
import rospy
import PyKDL
from numpy import *

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseWithCovarianceStamped

from cola2_msgs.msg import Setpoints
from sensor_msgs.msg import Joy, NavSatFix, Imu, NavSatStatus
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from ned_tools import NED utils, utils_ros
from cola2_msgs.msg import NavSts
from math import *
import numpy as np
import rosparam

# More imports
import numpy as np
import tf
import math


SAVITZKY_GOLAY_COEFFS = [0.2,  0.1,  0.0, -0.1, -0.2]

INIT_POSITION = [0.0, 0.0, 0.0]
INIT_ORIENTATION = [0.0, 0.0, 0.0, 1.0]


class SimNavSensors:
    """ This class is able to simulate the navigation sensors of s2 AUV """
    def __init__(self, name):

        """ Constructor """
        self.name = name

        # Load dynamic parameters
        self.get_config()
        self.ned = NED.NED(self.latitude, self.longitude, 0.0)
        self.odom = Odometry()
        self.orientation = np.zeros(4)
        self.altitude = -1.0

        # Initial vehicle pose
        vehicle_pose_t = tf.transformations.translation_matrix(np.array(INIT_POSITION))
        vehicle_pose_q = tf.transformations.quaternion_matrix(np.array(INIT_ORIENTATION))
        self.vehicle_pose = tf.transformations.concatenate_matrices(vehicle_pose_t, vehicle_pose_q)

        # Buffer to derive heading (ADIS IMU gives rates, not needed)
        self.imu_init = False
        self.heading_buffer = []
        self.savitzky_golay_coeffs = SAVITZKY_GOLAY_COEFFS

        # Tfs
        self.listener = tf.TransformListener()
        self.gps_tf_init = False

        # Create publishers
        self.pub_imu = rospy.Publisher('sensors/imu_raw', Imu, queue_size = 2)
        self.pub_gps = rospy.Publisher('sensors/gps_raw',NavSatFix,queue_size = 2)

        # Create subscribers to odometry and range
        rospy.Subscriber('dynamics/odometry', Odometry, self.update_odometry, queue_size = 1)

        # Init simulated sensor timers
        rospy.Timer(rospy.Duration(self.imu_period), self.pub_imu_callback)
        rospy.Timer(rospy.Duration(self.gps_period), self.pub_gps_callback)

        # Show message
        rospy.loginfo("%s: initialized", self.name)


    def update_odometry(self, odom):
        """ This method is a callback of the odometry message that comes
            from dynamics node """
        self.odom = odom

        vehicle_pose_t = tf.transformations.translation_matrix(np.array([self.odom.pose.pose.position.x,
                                                                           self.odom.pose.pose.position.y,
                                                                           self.odom.pose.pose.position.z]))
        vehicle_pose_q = tf.transformations.quaternion_matrix(np.array([self.odom.pose.pose.orientation.x,
                                                                          self.odom.pose.pose.orientation.y,
                                                                          self.odom.pose.pose.orientation.z,
                                                                          self.odom.pose.pose.orientation.w]))
        self.vehicle_pose = tf.transformations.concatenate_matrices(vehicle_pose_t, vehicle_pose_q)

        # Quaternion to Euler
        self.orientation = tf.transformations.euler_from_quaternion(
                                    [self.odom.pose.pose.orientation.x,
                                     self.odom.pose.pose.orientation.y,
                                     self.odom.pose.pose.orientation.z,
                                     self.odom.pose.pose.orientation.w])

    def pub_imu_callback(self, event):
        """ This method is a callback of a timer. This publishes imu data """
        # TODO: euler rate is not angular velocity!

        # Imu
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = self.robot_frame_id

        # Add some noise
        angle = [0.0]*3
        angle[0] = self.orientation[0] + np.random.normal(
                                      self.imu_drift[0] , self.imu_orientation_covariance_gen[0])
        angle[1] = self.orientation[1] + np.random.normal(
                                      self.imu_drift[1] , self.imu_orientation_covariance_gen[1])
        angle[2] = self.orientation[2] + np.random.normal(
                                      self.imu_drift[2], self.imu_orientation_covariance_gen[2])

        # Derive to obtain rates (only yaw is derived with savitzky_golay)
        if not self.imu_init:
            # Initialize heading buffer in order to apply a savitzky_golay derivation
            if len(self.heading_buffer) == 0:
                self.heading_buffer.append(angle[2])

            inc = utils.normalizeAngle(angle[2] - self.heading_buffer[-1])
            self.heading_buffer.append(self.heading_buffer[-1] + inc)

            if len(self.heading_buffer) == len(self.savitzky_golay_coeffs):
                self.imu_init = True

            # Euler derivation to roll and pitch, so:
            self.last_imu_orientation = angle
            self.last_imu_update = self.odom.header.stamp

        else:
            period = (self.odom.header.stamp - self.last_imu_update).to_sec()
            if period < 0.001:
                period = 0.001  # Minimum allowed period

            # For yaw rate we apply a savitzky_golay derivation
            inc = utils.normalizeAngle(angle[2] - self.heading_buffer[-1])
            self.heading_buffer.append(self.heading_buffer[-1] + inc)
            self.heading_buffer.pop(0)
            imu.angular_velocity.z = np.convolve(self.heading_buffer,
                                                 self.savitzky_golay_coeffs,
                                                 mode='valid') / period

            # TODO: Roll rate and Pitch rate should be also
            # savitzky_golay derivations?
            imu.angular_velocity.x = utils.normalizeAngle(
                           angle[0] - self.last_imu_orientation[0]) / period
            imu.angular_velocity.y = utils.normalizeAngle(
                           angle[1] - self.last_imu_orientation[1]) / period

            self.last_imu_orientation = angle
            self.last_imu_update = self.odom.header.stamp

            imu.angular_velocity_covariance = [0.0001,  0.,     0.,
                                               0.,      0.0001, 0.,
                                               0.,      0.,     0.0002]

            # Orientation
            angle_q = tf.transformations.quaternion_from_euler(angle[0],
                                                               angle[1],
                                                               angle[2])
            imu.orientation_covariance[0] = self.imu_orientation_covariance[0]
            imu.orientation_covariance[4] = self.imu_orientation_covariance[1]
            imu.orientation_covariance[8] = self.imu_orientation_covariance[2]

            imu.orientation.x = angle_q[0]
            imu.orientation.y = angle_q[1]
            imu.orientation.z = angle_q[2]
            imu.orientation.w = angle_q[3]

            # Publish
            self.pub_imu.publish(imu)


    def pub_gps_callback(self, event):
        """ This method is a callback of a timer. This publishes gps data """
        if self.gps_tf_init is False:
            try:
                rospy.logwarn("[%s]: waiting for %s transform", self.name, self.gps_frame_id)
                self.listener.waitForTransform(self.robot_frame_id,
                                               self.gps_frame_id,
                                               rospy.Time(),
                                               rospy.Duration(1.0))

                (trans, rot) = self.listener.lookupTransform(self.robot_frame_id, self.gps_frame_id, rospy.Time())
                rospy.loginfo("[%s]: transform for %s found", self.name, self.gps_frame_id)
                self.robot2gps = [trans, rot]
                self.gps_tf_init = True

            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException,
                    tf.Exception):
                rospy.logerr('[%s]: define TF for %s update!',
                             self.name,
                             self.gps_frame_id)
                return

        gps = NavSatFix()
        gps.header.stamp = rospy.Time.now()
        gps.header.frame_id = self.world_frame_id
        gps.status.status = NavSatStatus.STATUS_FIX;
        gps.status.service = NavSatStatus.SERVICE_GPS;
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN;
        gps.position_covariance[0] = self.gps_position_covariance[0]
        gps.position_covariance[4] = self.gps_position_covariance[1]
        gps.position_covariance[8] = self.gps_position_covariance[2]

        # Translate north and east to GPS sensor location
        robot2gps_tf = tf.transformations.quaternion_matrix(self.robot2gps[1])
        robot2gps_tf[:3, 3] = self.robot2gps[0]
        gps_point = tf.transformations.translation_from_matrix(tf.transformations.concatenate_matrices(self.vehicle_pose, robot2gps_tf))

        # Extract NED referenced pose
        north = (gps_point[0] + np.random.normal(self.gps_drift[0], self.gps_position_covariance_gen[0]))
        east = (gps_point[1] + np.random.normal(self.gps_drift[1], self.gps_position_covariance_gen[1]))

        # Convert coordinates
        lat, lon, h = self.ned.ned2geodetic(np.array([north, east, self.odom.pose.pose.position.z]))
        gps.latitude = lat
        gps.longitude = lon

        # Publish
        self.pub_gps.publish(gps)


    def get_config(self):
        """ Get config from param server """
        param_dict = {'latitude': "navigator/ned_latitude",
                      'longitude': "navigator/ned_longitude",
                      'world_frame_id': "frames/map",
                      'robot_frame_id': "frames/base_link",
                      'imu_frame_id': "frames/sensors/imu",
                      'gps_frame_id': "frames/sensors/gps",
                      'origin_suffix': "frames/sensors/origin_suffix",
                      'imu_period': "sim_nav_sensors/imu/period",
                      'gps_period': "sim_nav_sensors/gps/period",
                      'imu_drift': "sim_nav_sensors/imu/drift",
                      'gps_drift': "sim_nav_sensors/gps/drift",
                      'imu_orientation_covariance': "sim_nav_sensors/imu/orientation_covariance",
                      'gps_position_covariance': "sim_nav_sensors/gps/position_covariance",
                      'imu_orientation_covariance_gen': "sim_nav_sensors/imu/orientation_covariance_gen",
                      'gps_position_covariance_gen': "sim_nav_sensors/gps/position_covariance_gen"}

        if not utils_ros.getRosParams(self, param_dict, self.name):
            rospy.logfatal("%s: shutdown due to invalid config parameters!", self.name)
            exit(0)  # TODO: find a better way


def __compute_tf__(transform):
    r = PyKDL.Rotation.RPY(math.radians(transform[3]),
                           math.radians(transform[4]),
                           math.radians(transform[5]))
    v = PyKDL.Vector(transform[0], transform[1], transform[2])
    frame = PyKDL.Frame(r, v)
    return frame


if __name__ == '__main__':
    try:
        rospy.init_node('sim_sensors')
        sim_sensors = SimNavSensors(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
