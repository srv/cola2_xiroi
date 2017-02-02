#!/usr/bin/env python
"""@@This node is used to simulate Sparus II navigation sensors. It is only used
in simulation.@@"""

"""
Modified 11/2015
"""

# Basic ROS imports
import rospy
import PyKDL

# import msgs
from nav_msgs.msg import Odometry
from cola2_msgs.msg import LinkquestDvl
from cola2_msgs.msg import TeledyneExplorerDvl
from cola2_msgs.msg import FastraxIt500Gps
from cola2_msgs.msg import PressureSensor
from cola2_msgs.msg import EmusBms
from sensor_msgs.msg import Imu
from cola2_lib import cola2_lib, cola2_ros_lib, NED
from sensor_msgs.msg import Range

# More imports
import numpy as np
import tf
import math

SAVITZKY_GOLAY_COEFFS = [0.2,  0.1,  0.0, -0.1, -0.2]

class SimNavSensorsS2:
    """ This class is able to simulate the navigation sensors of s2 AUV """

    def __init__(self, name):
        """ Constructor """
        self.name = name

        # Load dynamic parameters
        self.simulate_altidude = True
        self.sea_bottom_depth = 20.0
        self.get_config()
        self.ned = NED.NED(self.latitude, self.longitude, 0.0)
        self.odom = Odometry()
        self.orientation = np.zeros(4)
        self.altitude = -1.0

        # Buffer to derive heading (ADIS IMU gives rates, not needed)
        self.imu_init = False
        self.heading_buffer = []
        self.savitzky_golay_coeffs = SAVITZKY_GOLAY_COEFFS

        # Create publishers
        self.pub_bms = rospy.Publisher('/cola2_safety/emus_bms',
                                       EmusBms,
                                       queue_size = 2)

        self.pub_imu = rospy.Publisher('/cola2_navigation/imu',
                                       Imu,
                                       queue_size = 2)

        self.pub_pressure = rospy.Publisher('/cola2_navigation/pressure_sensor',
                                            PressureSensor,
                                            queue_size = 2)

        self.pub_gps = rospy.Publisher('/cola2_navigation/fastrax_it_500_gps',
                                       FastraxIt500Gps,
                                       queue_size = 2)

        if self.dvl_type == "linkquest":
            self.pub_dvl = rospy.Publisher(
                                 '/cola2_navigation/linkquest_navquest600_dvl',
                                 LinkquestDvl,
                                 queue_size = 2)

        elif self.dvl_type == "rdi":
            self.pub_dvl = rospy.Publisher('/cola2_navigation/teledyne_explorer_dvl',
                                 TeledyneExplorerDvl,
                                 queue_size = 2)

        elif self.dvl_type == "none":
            rospy.loginfo("%s: not publishing dvl", self.name)
        else:
            rospy.logwarn("%s: unknown dvl type", self.name)

        # Create subscribers to odometry and range
        rospy.Subscriber(self.odom_topic_name, Odometry, self.update_odometry, queue_size = 1)
        rospy.Subscriber(self.altitude_range_topic_name, Range, self.update_altitude, queue_size = 1)

        # Init simulated sensor timers
        rospy.Timer(rospy.Duration(1.0), self.pub_bms_callback)
        rospy.Timer(rospy.Duration(self.imu_period), self.pub_imu_callback)
        if self.dvl_type == "linkquest":
            rospy.Timer(rospy.Duration(self.dvl_period), self.pub_linkquest_dvl_callback)
        elif self.dvl_type == "rdi":
            rospy.Timer(rospy.Duration(self.dvl_period), self.pub_rdi_dvl_callback)
        rospy.Timer(rospy.Duration(self.gps_period), self.pub_gps_callback)

        # Show message
        rospy.loginfo("%s: initialized", self.name)


    def update_odometry(self, odom):
        """ This method is a callback of the odometry message that comes
            from dynamics node """
        self.odom = odom

        # Quaternion to Euler
        self.orientation = tf.transformations.euler_from_quaternion(
                                    [self.odom.pose.pose.orientation.x,
                                     self.odom.pose.pose.orientation.y,
                                     self.odom.pose.pose.orientation.z,
                                     self.odom.pose.pose.orientation.w])


    def update_altitude(self, range_data):
        """ This method is a callback of the range input message """
        self.simulate_altidude = False
        self.altitude = range_data.range


    def pub_bms_callback(self, event):
        """ This method is a callback of a timer. This publishes bms data """
        msg = EmusBms()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'sparus2'
        msg.voltage = 29.0
        msg.minCellVoltage = 4.0
        self.pub_bms.publish(msg)


    def pub_imu_callback(self, event):
        """ This method is a callback of a timer. This publishes imu and pressure
            sensor data """  # TODO: euler rate is not angular velocity!
        # First publish pressure sensor
        msg = PressureSensor()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'pressure_sensor'
        msg.temperature = 25.0
        msg.pressure = self.odom.pose.pose.position.z * self.water_density * 9.81 / 100000.0
        self.pub_pressure.publish(msg)

        # Imu
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = 'adis_imu'

        # Add some noise
        roll = self.orientation[0] + np.random.normal(
                                      0.0, self.imu_orientation_covariance[0])
        pitch = self.orientation[1] + np.random.normal(
                                      0.0, self.imu_orientation_covariance[1])
        yaw = self.orientation[2] + np.random.normal(
                                      0.0, self.imu_orientation_covariance[2])

        # Rotate as necessary
        vehicle_rpy = PyKDL.Rotation.RPY(roll, pitch, yaw)
        imu_orientation = self.imu_tf.M * vehicle_rpy

        # Get RPY
        angle = imu_orientation.GetRPY()

        # Derive to obtain rates
        if not self.imu_init:
            # Initialize heading buffer in order to apply a
            # savitzky_golay derivation
            if len(self.heading_buffer) == 0:
                self.heading_buffer.append(angle[2])

            inc = cola2_lib.normalizeAngle(angle[2] - self.heading_buffer[-1])
            self.heading_buffer.append(self.heading_buffer[-1] + inc)

            if len(self.heading_buffer) == len(self.savitzky_golay_coeffs):
                self.imu_init = True

            # Euler derivation to roll and pitch, so:
            self.last_imu_orientation = angle
            self.last_imu_update = imu.header.stamp

        else:
            period = (imu.header.stamp - self.last_imu_update).to_sec()
            if period < 0.001:
                period = 0.001  # Minimum allowed period

            # For yaw rate we apply a savitzky_golay derivation
            inc = cola2_lib.normalizeAngle(angle[2] - self.heading_buffer[-1])
            self.heading_buffer.append(self.heading_buffer[-1] + inc)
            self.heading_buffer.pop(0)
            imu.angular_velocity.z = np.convolve(self.heading_buffer,
                                              self.savitzky_golay_coeffs,
                                              mode='valid') / period

            # TODO: Roll rate and Pitch rate should be also
            # savitzky_golay derivations?
            imu.angular_velocity.x = cola2_lib.normalizeAngle(
                           angle[0] - self.last_imu_orientation[0]) / period
            imu.angular_velocity.y = cola2_lib.normalizeAngle(
                           angle[1] - self.last_imu_orientation[1]) / period

            self.last_imu_orientation = angle
            self.last_imu_update = imu.header.stamp

            imu.angular_velocity_covariance = [0.0001,  0.,     0.,
                                               0.,      0.0001, 0.,
                                               0.,      0.,     0.0002]

            # Euler to Quaternion
            angle = tf.transformations.quaternion_from_euler(angle[0],
                                                             angle[1],
                                                             angle[2])
            imu.orientation.x = angle[0]
            imu.orientation.y = angle[1]
            imu.orientation.z = angle[2]
            imu.orientation.w = angle[3]

            imu.orientation_covariance[0] = self.imu_orientation_covariance[0]
            imu.orientation_covariance[4] = self.imu_orientation_covariance[1]
            imu.orientation_covariance[8] = self.imu_orientation_covariance[2]

            #rospy.loginfo("%s: publishing imu", self.name)
            self.pub_imu.publish(imu)

        # Publish TF
        imu_tf = tf.TransformBroadcaster()
        o = tf.transformations.quaternion_from_euler(
                                    math.radians(self.imu_tf_array[3]),
                                    math.radians(self.imu_tf_array[4]),
                                    math.radians(self.imu_tf_array[5]),
                                    'sxyz')
        imu_tf.sendTransform((self.imu_tf_array[0],
                              self.imu_tf_array[1],
                              self.imu_tf_array[2]), o, imu.header.stamp,
                             'adis_imu_from_sim_nav_sensors', 'sparus2')


    def pub_linkquest_dvl_callback(self, event):
        """ This method is a callback of a timer. This publishes linkquest dvl data """
        # TODO: Fix tf's
        dvl = LinkquestDvl()
        dvl.header.stamp = rospy.Time.now()
        dvl.header.frame_id = 'linkquest_navquest600_dvl_from_sim_nav_sensors'

        # Add noise
        v_dvl = np.zeros(3)
        v_dvl[0] = (self.odom.twist.twist.linear.x +
                    np.random.normal(0.0, self.dvl_velocity_covariance[0]))
        v_dvl[1] = (self.odom.twist.twist.linear.y +
                    np.random.normal(0.0, self.dvl_velocity_covariance[1]))
        v_dvl[2] = (self.odom.twist.twist.linear.z +
                    np.random.normal(0.0, self.dvl_velocity_covariance[2]))

        # Velocity is computed at the gravity center but we want the velocity
        # at the sensor. Then: Vdvl = Vg500 + (v.ang.z x dist(dvl->g500))
        angular_velocity = np.array([self.odom.twist.twist.angular.x,
                                     self.odom.twist.twist.angular.y,
                                     self.odom.twist.twist.angular.z])
        distance = np.array([-0.41327, 0.0, 0.09229])

        v_dvl = v_dvl + np.cross(angular_velocity, distance)

        # Simulate the DVL rotation
        vel = PyKDL.Vector(v_dvl[0],
                           v_dvl[1],
                           v_dvl[2])
        R = PyKDL.Rotation.RPY(0.0, 0.0, -math.pi/4.0)
        vel_rot = R.Inverse() * vel

        v_dvl[0] = -vel_rot[0]
        v_dvl[1] = -vel_rot[1]
        v_dvl[2] = -vel_rot[2]

        # Compose message
        dvl.velocityInstFlag = 1
        dvl.velocityInst[0] = v_dvl[0]
        dvl.velocityInst[1] = v_dvl[1]
        dvl.velocityInst[2] = v_dvl[2]
        dvl.altitude = self.altitude
        for beam in range(4):
            dvl.dataGood[beam] = 1
            dvl.altitudeBeam[beam] = self.altitude

        #rospy.loginfo("%s: publishing dvl", self.name)
        self.pub_dvl.publish(dvl)

        # Publish TF
        dvl_tf = tf.TransformBroadcaster()
        o = tf.transformations.quaternion_from_euler(
                                    math.radians(self.dvl_tf_array[3]),
                                    math.radians(self.dvl_tf_array[4]),
                                    math.radians(self.dvl_tf_array[5]),
                                    'sxyz')
        dvl_tf.sendTransform((self.dvl_tf_array[0],
                              self.dvl_tf_array[1],
                              self.dvl_tf_array[2]), o, dvl.header.stamp,
                              'linkquest_navquest600_dvl_from_sim_nav_sensors', 'sparus2')


    def pub_rdi_dvl_callback(self, event):
        """ This method is a callback of a timer. This publishes linkquest rdi data """
        # TODO: Fix tf's
        dvl = TeledyneExplorerDvl()
        dvl.header.stamp = rospy.Time.now()
        dvl.header.frame_id = 'teledyne_explorer_dvl'

        # Add noise
        v_dvl = np.zeros(3)
        v_dvl[0] = (self.odom.twist.twist.linear.x +
                    np.random.normal(0.0, self.dvl_velocity_covariance[0]))
        v_dvl[1] = (self.odom.twist.twist.linear.y +
                    np.random.normal(0.0, self.dvl_velocity_covariance[1]))
        v_dvl[2] = (self.odom.twist.twist.linear.z +
                    np.random.normal(0.0, self.dvl_velocity_covariance[2]))

        # Velocity is computed at the gravity center but we want the velocity
        # at the sensor
        angular_velocity = np.array([self.odom.twist.twist.angular.x,
                                     self.odom.twist.twist.angular.y,
                                     self.odom.twist.twist.angular.z])
        distance = np.array([-0.41327, 0.0, 0.09229])

        #rospy.loginfo("%s: velocity[0] without translation: %s", self.name, v_dvl[0])
        #rospy.loginfo("%s: velocity[1] without translation: %s", self.name, v_dvl[1])
        #rospy.loginfo("%s: velocity[2] without translation: %s", self.name, v_dvl[2])

        v_dvl = v_dvl + np.cross(angular_velocity, distance)

        #rospy.loginfo("%s: velocity[0] with translation: %s", self.name, v_dvl[0])
        #rospy.loginfo("%s: velocity[1] with translation: %s", self.name, v_dvl[1])
        #rospy.loginfo("%s: velocity[2] with translation: %s", self.name, v_dvl[2])

        # Rotate the DVL
        R = PyKDL.Rotation.RPY(math.pi, 0.0, math.pi * 3.0 / 4.0)
        dvl_data = PyKDL.Vector(v_dvl[0], v_dvl[1], v_dvl[2])
        dvl_data = R.Inverse() * dvl_data
        dvl.bi_x_axis = dvl_data[0]
        dvl.bi_y_axis = dvl_data[1]
        dvl.bi_z_axis = dvl_data[2]
        dvl.bi_status = "A"

        # If simulated altitude, compute it
        if self.simulate_altidude:
            self.altitude = self.sea_bottom_depth - self.odom.pose.pose.position.z
            # print 'Altitude: ', self.altitude
            if self.altitude < 0.5:
                self.altitude = -1.0

        dvl.bd_range = self.altitude
        self.pub_dvl.publish(dvl)

        # Publish TF
        dvl_tf = tf.TransformBroadcaster()
        o = tf.transformations.quaternion_from_euler(
                                    math.radians(self.dvl_tf_array[3]),
                                    math.radians(self.dvl_tf_array[4]),
                                    math.radians(self.dvl_tf_array[5]),
                                    'sxyz')
        dvl_tf.sendTransform((self.dvl_tf_array[0],
                              self.dvl_tf_array[1],
                              self.dvl_tf_array[2]), o, dvl.header.stamp,
                              'teledyne_explorer_dvl_from_sim_nav_sensors', 'sparus2')


    def pub_gps_callback(self, event):
        """ This method is a callback of a timer. This publishes gps data """
        # Publish GPS data only near to surface
        gps = FastraxIt500Gps()
        gps.header.stamp = rospy.Time.now()
        gps.header.frame_id = 'fastrax_it_500_gps'
        north = (self.odom.pose.pose.position.x +
                 np.random.normal(0.0, self.gps_position_covariance[0]))
        east = (self.odom.pose.pose.position.y +
                np.random.normal(0.0, self.gps_position_covariance[1]))
        gps.north = north
        gps.east = east
        if self.odom.pose.pose.position.z < 0.5:
            gps.data_quality = 1
        else:
            gps.data_quality = 0
        gps.h_dop = 0.98
        gps.v_dop = 1.37

        lat, lon, h = self.ned.ned2geodetic(np.array([north, east, 0.0]))
        deg_lat, deg_lon = NED.degree2DegreeMinute(lat, lon)

        if deg_lat < 0.0:
            gps.latitude_hemisphere = 1
            deg_lat = -deg_lat
        else:
            gps.latitude_hemisphere = 0

        if deg_lon < 0.0:
            gps.longitude_hemisphere = 2
            deg_lon = -deg_lon
        else:
            gps.longitude_hemisphere = 3

        gps.latitude = deg_lat
        gps.longitude = deg_lon

        #rospy.loginfo("%s: publishing gps", self.name)
        self.pub_gps.publish(gps)

        # Publish GPS TF
        gps_tf = tf.TransformBroadcaster()
        o = tf.transformations.quaternion_from_euler(
                                math.radians(self.gps_tf_array[3]),
                                math.radians(self.gps_tf_array[4]),
                                math.radians(self.gps_tf_array[5]),
                                'sxyz')
        gps_tf.sendTransform((self.gps_tf_array[0],
                              self.gps_tf_array[1],
                              self.gps_tf_array[2]), o, gps.header.stamp,
                              'gps_from_sim_nav_sensors', 'sparus2')


    def get_config(self):
        """ Get config from param server """
        if rospy.has_param("vehicle_name"):  # This parameter is in dynamics yaml
            self.vehicle_name = rospy.get_param('vehicle_name')
        else:
            rospy.logfatal("%s: vehicle_name parameter not found", self.name)
            exit(0)  # TODO: find a better way

        param_dict = {'latitude': "dynamics/" + self.vehicle_name + "/ned_origin_latitude",
                      'longitude': "dynamics/" + self.vehicle_name + "/ned_origin_longitude",
                      'odom_topic_name': "dynamics/" + self.vehicle_name + "/odom_topic_name",
                      'altitude_range_topic_name': "dynamics/" + self.vehicle_name + "/altitude_range_topic_name",
                      'world_frame_id': "dynamics/" + self.vehicle_name + "/world_frame_id",  # TODO: Shouldn't this be general for all nodes?
                      'dvl_type': "sim_nav_sensors/dvl_type",
                      'water_density': "sim_nav_sensors/water_density",
                      'imu_tf_array': "sim_nav_sensors/imu/tf",
                      'dvl_tf_array': "sim_nav_sensors/dvl/tf",
                      'gps_tf_array': "sim_nav_sensors/gps/tf",
                      'imu_period': "sim_nav_sensors/imu/period",
                      'dvl_period': "sim_nav_sensors/dvl/period",
                      'sea_bottom_depth': "sea_bottom_depth",
                      'gps_period': "sim_nav_sensors/gps/period",
                      'imu_orientation_covariance': "sim_nav_sensors/imu/orientation_covariance",
                      'dvl_velocity_covariance': "sim_nav_sensors/dvl/velocity_covariance",
                      'gps_position_covariance': "sim_nav_sensors/gps/position_covariance"}

        if not cola2_ros_lib.getRosParams(self, param_dict, self.name):
            rospy.logfatal("%s: shutdown due to invalid config parameters!", self.name)
            exit(0)  # TODO: find a better way
        self.imu_tf = __compute_tf__(np.array(self.imu_tf_array) * -1.0)
        self.dvl_tf = __compute_tf__(np.array(self.dvl_tf_array) * -1.0)


def __compute_tf__(transform):
    r = PyKDL.Rotation.RPY(math.radians(transform[3]),
                           math.radians(transform[4]),
                           math.radians(transform[5]))
    v = PyKDL.Vector(transform[0], transform[1], transform[2])
    frame = PyKDL.Frame(r, v)
    return frame


if __name__ == '__main__':
    try:
        rospy.init_node('sim_nav_sensors_s2')
        sim_nav_sensors = SimNavSensorsS2(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
