#!/usr/bin/env python

# Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


"""@@>Takes raw data from Adis IMU and converts them to orientation, angular velocities and linear accelerations.<@@"""

import roslib
import rospy
import rospkg
from cola2_lib.utils.angles import wrap_angle
from cola2_lib.rosutils.transform_handler import TransformHandler
from cola2_msgs.msg import BodyVelocityReq
from sensor_msgs.msg import Imu, MagneticField
from cola2_xiroi.msg import ImuAngleEstimatorInfo
from std_srvs.srv import Empty, EmptyResponse
import tf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import datetime
from numpy import *
import PyKDL
import sys
import message_filters

def rot_from_euler(euler):
    """ Compute rotation matrix from euler angles """
    cr = cos(euler[0, 0])
    sr = sin(euler[0, 0])
    cp = cos(euler[1, 0])
    sp = sin(euler[1, 0])
    cy = cos(euler[2, 0])
    sy = sin(euler[2, 0])

    out = matrix([[cp*cy, -cr*sy+sr*sp*cy, sr*sy+cr*sp*cy],
                  [cp*sy, cr*cy+sr*sp*sy, -sr*cy+cr*sp*sy],
                  [-sp, sr*cp, cr*cp]])

    return out


def euler_from_rot(rot):
    """ Compute euler angles from rotation matrix """
    out = matrix([[math.atan2(rot[2, 1], rot[2, 2])],
                  [math.atan2(-rot[2, 0], (rot[0, 0] ** 2.0 + rot[1, 0] ** 2.0) ** 0.5)],
                  [math.atan2(rot[1, 0], rot[0, 0])]])

    return out


def skew_symmetric(data):
    """ Compute skew symmetric matrix """
    out = matrix([[0, -data[2, 0], data[1, 0]],
                  [data[2, 0], 0, -data[0, 0]],
                  [-data[1, 0], data[0, 0], 0]])

    return out


class SUM_HEAP:
    def __init__(self, n_elem):
        """ Constructor """
        self.n_elem = n_elem
        self.layers = int(ceil(log2(n_elem + 1)))
        self.n_elem_max = 2 ** self.layers
        self.buffer = [0.0] * 2 * self.n_elem_max
        self.elements_in_container = 0
        self.index = 1

    def insert(self, data):
        # Compute index and store data
        index = self.index + self.n_elem_max
        self.buffer[index] = data

        # Compute layers below
        for i in range(self.layers):
            index = index // 2
            self.buffer[index] = self.buffer[2 * index] + self.buffer[2 * index + 1]

        # Keep track of index
        self.index = self.index + 1
        if self.index > self.n_elem:
            self.index = 1

        # Keep track of size
        if self.elements_in_container < self.n_elem:
            self.elements_in_container = self.elements_in_container + 1

    def size(self):
        return self.elements_in_container

    def sum(self):
        return self.buffer[1]

    def reset(self):
        self.buffer = [0.0] * 2 * self.n_elem_max
        self.elements_in_container = 0
        self.index = 1


class IMU_ANGLE_ESTIMATOR:
    """ This is the main class of the imu_angle_estimator.py node, which
        is able to compute imu output from mag, acc and gyros. It is also able
        to compute soft and hard iron calibrations and corrections """

    def __init__(self, name):
        """ Init the class """
        self.name = name
        self.tfh = TransformHandler()

        # Some calibration parameters
        self.min_gyro_bias_buffer_size = 100
        self.filter_divergence_buffer_size = 1000
        self.acceleration_limit = 0.20
        self.rates_limit = 0.5
        self.filter_reduction_factor = 0.5
        self.angle_covariance = 0.1
        self.angular_velocity_covariance = 0.1
        self.acceleration_covariance = 0.1

        # Calibration rate
        self.rate = 10

        # Get config
        self.get_config()

        # Init vars
        # Used to compute an euler derivative
        self.past_time = rospy.Time.now()
        self.magnetometer_enabled = True
        self.freeze_gyro_bias_flag = False
        self.filter_init = False

        # This is used to compute an euler derivative of the tilt corrected angles to get euler rates
        self.past_tilt_roll = 0.0
        self.past_tilt_pitch = 0.0
        self.past_tilt_yaw = 0.0

        # This is used to compute a mean filter for the angular velocities bias
        self.bias_roll = SUM_HEAP(self.max_gyro_bias_buffer_size)
        self.bias_pitch = SUM_HEAP(self.max_gyro_bias_buffer_size)
        self.bias_yaw = SUM_HEAP(self.max_gyro_bias_buffer_size)

        # This is used to check divergence
        self.merged_minus_tilt_roll_vector = []
        self.merged_minus_tilt_pitch_vector = []
        self.merged_minus_tilt_yaw_vector = []
        self.divergence_factor = 1.0

        # This is used when filtering
        self.merged_roll = 0.0
        self.merged_pitch = 0.0
        self.merged_yaw = 0.0

        # Init calibration vars
        self.ax_calib_vector = []
        self.ay_calib_vector = []
        self.az_calib_vector = []
        self.mx_calib_vector = []
        self.my_calib_vector = []
        self.mz_calib_vector = []
        self.filtered_tilt_corrected_mx_calib_vector = []
        self.filtered_tilt_corrected_my_calib_vector = []
        self.filtered_tilt_corrected_mz_calib_vector = []
        self.magnetometer_calibration_flag = 0

        # Create publisher
        self.pub = rospy.Publisher('~imu',
                                   Imu,
                                   queue_size = 2)
        self.info_pub = rospy.Publisher('~imu_info',
                                        ImuAngleEstimatorInfo,
                                        queue_size = 2)
        self.pub_body_velocity_req = rospy.Publisher( rospy.get_namespace() + 'controller/body_velocity_req',
                                                     BodyVelocityReq,
                                                     queue_size = 2)

        # Create services
        self.calibrate_magnetometer_srv = rospy.Service(
                         '~calibrate_magnetometer',
                         Empty, self.calibrate_magnetometer)
        self.stop_magnetometer_calibration_srv = rospy.Service(
                         '~stop_magnetometer_calibration',
                         Empty, self.stop_magnetometer_calibration)
        self.enable_magnetometer_srv = rospy.Service(
                         '~enable_magnetometer',
                         Empty, self.enable_magnetometer)
        self.disable_magnetometer_srv = rospy.Service(
                         '~disable_magnetometer',
                         Empty, self.disable_magnetometer)
        self.freeze_gyro_bias_srv = rospy.Service(
                         '~freeze_gyro_bias',
                         Empty, self.freeze_gyro_bias)
        self.unfreeze_gyro_bias_srv = rospy.Service(
                         '~unfreeze_gyro_bias',
                         Empty, self.unfreeze_gyro_bias)
        self.reload_configuration_srv = rospy.Service(
                         '~reload_configuration',
                         Empty, self.reload_configuration)

        # Initialized
        rospy.loginfo("%s: initialized", self.name)


    def new_data_callback(self, imu_msg, mag_msg):
        """ This is the callback of the imu message that comes from the
            driver """
        # Rotate accelerometer
        acc_vector = matrix([[imu_msg.linear_acceleration.x],
                             [imu_msg.linear_acceleration.y],
                             [imu_msg.linear_acceleration.z]])
        acc_vector = self.imu_rotation * acc_vector

        # Apply calibration and rotate magnetometer
        mag_input = matrix([[mag_msg.magnetic_field.x],
                            [mag_msg.magnetic_field.y],
                            [mag_msg.magnetic_field.z]])
        mag_calib = self.soft_iron * mag_input + self.hard_iron
        mag_vector = matrix([[mag_calib[0, 0]],
                             [mag_calib[1, 0]],
                             [mag_calib[2, 0]]])
        mag_vector = self.imu_rotation * mag_vector

        # Rotate gyro
        gyr_vector = matrix([[imu_msg.angular_velocity.x],
                             [imu_msg.angular_velocity.y],
                             [imu_msg.angular_velocity.z]])
        gyr_vector = self.imu_rotation * gyr_vector

        # Store data to self
        self.ax = acc_vector[0, 0]
        self.ay = acc_vector[1, 0]
        self.az = acc_vector[2, 0]
        self.mx = mag_vector[0, 0]
        self.my = mag_vector[1, 0]
        self.mz = mag_vector[2, 0]
        self.gx = gyr_vector[0, 0]
        self.gy = gyr_vector[1, 0]
        self.gz = gyr_vector[2, 0]

        # Callback produces computation
        try:
            self.compute()
        except:
            e = sys.exc_info()[1]
            rospy.logerr("%s: unexpected error: %s", self.name, str(e))

        # Store data for calibration
        if self.magnetometer_calibration_flag == 1:
            # Rotate magnetometer. No calibration here
            mag_vector = matrix([[mag_msg.magnetic_field.x],
                                 [mag_msg.magnetic_field.y],
                                 [mag_msg.magnetic_field.z]])
            mag_vector = self.imu_rotation * mag_vector

            # Store data to self
            self.ax_calib_vector.append(acc_vector[0, 0])
            self.ay_calib_vector.append(acc_vector[1, 0])
            self.az_calib_vector.append(acc_vector[2, 0])
            self.mx_calib_vector.append(mag_vector[0, 0])
            self.my_calib_vector.append(mag_vector[1, 0])
            self.mz_calib_vector.append(mag_vector[2, 0])


    def compute(self):
        """ Make an iteration """
        # Initialize info message
        info_msg = ImuAngleEstimatorInfo()

        # Initialize and add some info to the string message
        status = ''
        if not self.magnetometer_enabled:
            status += 'Magnetometer has been disabled. '
        if self.freeze_gyro_bias_flag:
            status += 'Gyro bias computation has been frozen. '

        # Save acceleration, magnetometer and gyro data after rotation and calibration
        info_msg.ax = self.ax
        info_msg.ay = self.ay
        info_msg.az = self.az
        info_msg.mx = self.mx
        info_msg.my = self.my
        info_msg.mz = self.mz
        info_msg.gx = self.gx
        info_msg.gy = self.gy
        info_msg.gz = self.gz

        # Get period
        new_time = rospy.Time.now()
        real_period = (new_time - self.past_time).to_sec()
        self.past_time = new_time
        info_msg.period = real_period
        if real_period < 0.02:
            status += 'Period too small: ' + str(real_period) + ' seconds. Clipping at 0.02 seconds. '
            real_period = 0.02
        elif real_period > 0.13:
            status += 'Period too big: ' + str(real_period) + ' seconds. Clipping at 0.13 seconds. '
            real_period = 0.13

        # Get tilt compensated euler angles (input has to be in NED). Leveling and tilt correction
        tilt_roll = math.atan2(-self.ay, -self.az)  # Roll comes from direct atan2 (numpy atan2 works even if both inputs are 0.0 -> output 0.0 which is ok)
        #tilt_pitch = math.atan2(-self.ax, self.ay * math.sin(tilt_roll) + self.az * math.cos(tilt_roll))  # Pitch is a bit more tricky
        den = -self.ay * math.sin(tilt_roll) - self.az * math.cos(tilt_roll)  # This is to have the pitch from pi/2 to -pi/2 (to force unique output)
        if den != 0.0:
            tilt_pitch = math.atan(self.ax / den)
        else:
            if self.ax == 0.0:
                tilt_pitch = 0.0  # Not enough information
            elif self.ax < 0.0:
                tilt_pitch = -math.pi * 0.5  # -pi/2
            else:
                tilt_pitch = math.pi * 0.5  # pi/2
        rot1 = matrix([[math.cos(tilt_pitch), 0.0, math.sin(tilt_pitch)], [0.0, 1.0, 0.0], [-math.sin(tilt_pitch), 0.0, math.cos(tilt_pitch)]])  # Pitch rotation
        rot2 = matrix([[1.0, 0.0, 0.0], [0.0, math.cos(tilt_roll), -math.sin(tilt_roll)], [0.0, math.sin(tilt_roll), math.cos(tilt_roll)]])  # Roll rotation
        rot = rot1 * rot2  # Roll and pitch rotation
        corrected_m = rot * matrix([[self.mx], [self.my], [self.mz]])  # Rotate magnetometer to have zero roll and zero pitch
        tilt_yaw = math.atan2(-corrected_m[1, 0], corrected_m[0, 0])  # Get yaw from tilt corrected magnetometer
        # TODO: Now declination is only in the navigator
        # tilt_yaw = wrap_angle(tilt_yaw + self.declination)  # Added declination correction
        info_msg.tilt_roll = tilt_roll
        info_msg.tilt_pitch = tilt_pitch
        info_msg.tilt_yaw = tilt_yaw

        # Euler rates from tilt angles
        rate_tilt_roll = wrap_angle(tilt_roll - self.past_tilt_roll) / real_period  # Simple Euler derivative
        rate_tilt_pitch = wrap_angle(tilt_pitch - self.past_tilt_pitch) / real_period
        rate_tilt_yaw = wrap_angle(tilt_yaw - self.past_tilt_yaw) / real_period
        self.past_tilt_roll = tilt_roll
        self.past_tilt_pitch = tilt_pitch
        self.past_tilt_yaw = tilt_yaw
        info_msg.rate_tilt_roll = rate_tilt_roll
        info_msg.rate_tilt_pitch = rate_tilt_pitch
        info_msg.rate_tilt_yaw = rate_tilt_yaw

        # Compute angular velocities from euler rates
        tilt_angular_velocity_x = rate_tilt_roll - rate_tilt_yaw * math.sin(tilt_pitch)
        tilt_angular_velocity_y = rate_tilt_pitch * math.cos(tilt_roll) + rate_tilt_yaw * math.cos(tilt_pitch) * math.sin(tilt_roll)
        tilt_angular_velocity_z = rate_tilt_yaw * math.cos(tilt_roll) * math.cos(tilt_pitch) - rate_tilt_pitch * math.sin(tilt_roll)
        info_msg.tilt_angular_velocity_x = tilt_angular_velocity_x
        info_msg.tilt_angular_velocity_y = tilt_angular_velocity_y
        info_msg.tilt_angular_velocity_z = tilt_angular_velocity_z

        # Euler rate bias, averaging filter and corrected euler rates
        bias_roll_from_this_iteration = self.gx - tilt_angular_velocity_x
        bias_pitch_from_this_iteration = self.gy - tilt_angular_velocity_y
        bias_yaw_from_this_iteration = self.gz - tilt_angular_velocity_z
        info_msg.bias_roll_from_this_iteration = bias_roll_from_this_iteration
        info_msg.bias_pitch_from_this_iteration = bias_pitch_from_this_iteration
        info_msg.bias_yaw_from_this_iteration = bias_yaw_from_this_iteration

        # Add new data to bias estimation buffers
        if self.magnetometer_enabled and not self.freeze_gyro_bias_flag:
            self.bias_roll.insert(bias_roll_from_this_iteration)
            self.bias_pitch.insert(bias_pitch_from_this_iteration)
            self.bias_yaw.insert(bias_yaw_from_this_iteration)

        # Check size
        if self.bias_roll.size() < self.max_gyro_bias_buffer_size:
            status += 'Gyro bias buffers not full yet. '

        # Compute bias as a mean of bias buffers
        if self.bias_roll.size() > 0:
            den_inv = 1.0 / float(self.bias_roll.size())
            bias_roll = self.bias_roll.sum() * den_inv
            bias_pitch = self.bias_pitch.sum() * den_inv
            bias_yaw = self.bias_yaw.sum() * den_inv
        else:
            bias_roll = 0.0
            bias_pitch = 0.0
            bias_yaw = 0.0
        info_msg.bias_roll = bias_roll
        info_msg.bias_pitch = bias_pitch
        info_msg.bias_yaw = bias_yaw
        info_msg.bias_buffer_size = self.bias_roll.size()

        # Filter and merge data with complementary filter instead of an EKF
        if not self.filter_init:
            # Handle first iteration. Initialize merged values with tilt ones
            self.merged_roll = tilt_roll
            self.merged_pitch = tilt_pitch
            self.merged_yaw = tilt_yaw
            self.filter_init = True
        else:
            # Compute corrected angular velocities
            corrected_angular_velocity_x = self.gx - bias_roll
            corrected_angular_velocity_y = self.gy - bias_pitch
            corrected_angular_velocity_z = self.gz - bias_yaw
            info_msg.corrected_angular_velocity_x = corrected_angular_velocity_x
            info_msg.corrected_angular_velocity_y = corrected_angular_velocity_y
            info_msg.corrected_angular_velocity_z = corrected_angular_velocity_z

            # If there are huge accelerations or rotations, use more gyros
            fade_factor = 0.7 + 0.3 * (float(self.bias_roll.size()) / float(self.min_gyro_bias_buffer_size)) ** 0.5
            if fade_factor > 1.0:
                fade_factor = 1.0
            merging_filter_coefficient = self.merging_filter_coefficient * fade_factor * self.divergence_factor
            if abs((self.ax ** 2.0 + self.ay ** 2.0 + self.az ** 2.0) ** 0.5 - 9.80665) > self.acceleration_limit:
                merging_filter_coefficient = 1.0 - self.filter_reduction_factor * (1.0 - merging_filter_coefficient)
            if abs(corrected_angular_velocity_x) + abs(corrected_angular_velocity_y) + abs(corrected_angular_velocity_z) > self.rates_limit:
                merging_filter_coefficient = 1.0 - self.filter_reduction_factor * (1.0 - merging_filter_coefficient)
            if not self.magnetometer_enabled:  # If magnetometer is disabled, use only gyros
                merging_filter_coefficient = 1.0
            info_msg.merging_filter_coefficient = merging_filter_coefficient

            # Integrate gyro corrected angular velocities using Rodriques equation
            euler_angles = matrix([[self.merged_roll],
                                   [self.merged_pitch],
                                   [self.merged_yaw]])
            corrected_w = matrix([[corrected_angular_velocity_x],
                                  [corrected_angular_velocity_y],
                                  [corrected_angular_velocity_z]])
            C_old = rot_from_euler(euler_angles)
            alpha_ib_b = real_period * corrected_w
            mag_alpha = (alpha_ib_b.transpose() * alpha_ib_b)[0, 0] ** 0.5
            Alpha_ib_b = skew_symmetric(alpha_ib_b)
            if mag_alpha > 1.0e-8:
                C_new_old = matrix(identity(3)) + sin(mag_alpha) / mag_alpha * Alpha_ib_b + (1.0 - cos(mag_alpha)) / mag_alpha ** 2.0 * Alpha_ib_b * Alpha_ib_b
            else:
                C_new_old = matrix(identity(3)) + Alpha_ib_b
            C_new = C_old * C_new_old
            euler_angles_new = euler_from_rot(C_new)
            gyro_integrated_roll = euler_angles_new[0, 0]
            gyro_integrated_pitch = euler_angles_new[1, 0]
            gyro_integrated_yaw = euler_angles_new[2, 0]
            info_msg.gyro_integrated_roll = gyro_integrated_roll
            info_msg.gyro_integrated_pitch = gyro_integrated_pitch
            info_msg.gyro_integrated_yaw = gyro_integrated_yaw

            # Compute gyro innovation
            gyro_roll_innovation = wrap_angle(gyro_integrated_roll - self.merged_roll)
            gyro_pitch_innovation = wrap_angle(gyro_integrated_pitch - self.merged_pitch)
            gyro_yaw_innovation = wrap_angle(gyro_integrated_yaw - self.merged_yaw)
            info_msg.gyro_roll_innovation = gyro_roll_innovation
            info_msg.gyro_pitch_innovation = gyro_pitch_innovation
            info_msg.gyro_yaw_innovation = gyro_yaw_innovation

            # Compute tilt innovation
            tilt_roll_innovation = wrap_angle(tilt_roll - self.merged_roll)
            tilt_pitch_innovation = wrap_angle(tilt_pitch - self.merged_pitch)
            tilt_yaw_innovation = wrap_angle(tilt_yaw - self.merged_yaw)
            info_msg.tilt_roll_innovation = tilt_roll_innovation
            info_msg.tilt_pitch_innovation = tilt_pitch_innovation
            info_msg.tilt_yaw_innovation = tilt_yaw_innovation

            # Compute innovation
            roll_innovation = merging_filter_coefficient * gyro_roll_innovation + (1.0 - merging_filter_coefficient) * tilt_roll_innovation
            pitch_innovation = merging_filter_coefficient * gyro_pitch_innovation + (1.0 - merging_filter_coefficient) * tilt_pitch_innovation
            yaw_innovation = merging_filter_coefficient * gyro_yaw_innovation + (1.0 - merging_filter_coefficient) * tilt_yaw_innovation
            info_msg.roll_innovation = roll_innovation
            info_msg.pitch_innovation = pitch_innovation
            info_msg.yaw_innovation = yaw_innovation

            # Compute new state (merged_xxxx may not follow the -180 to 180 for roll, -90 to 90 for pitch and -180 to 180 for yaw convention, but as the output will be a quaternion, this doesn't matter)
            self.merged_roll = wrap_angle(self.merged_roll + roll_innovation)
            self.merged_pitch = wrap_angle(self.merged_pitch + pitch_innovation)
            self.merged_yaw = wrap_angle(self.merged_yaw + yaw_innovation)
            info_msg.merged_roll = self.merged_roll
            info_msg.merged_pitch = self.merged_pitch
            info_msg.merged_yaw = self.merged_yaw
            merged_minus_tilt_roll = wrap_angle(self.merged_roll - tilt_roll)
            merged_minus_tilt_pitch = wrap_angle(self.merged_pitch - tilt_pitch)
            merged_minus_tilt_yaw = wrap_angle(self.merged_yaw - tilt_yaw)
            info_msg.merged_minus_tilt_roll = merged_minus_tilt_roll
            info_msg.merged_minus_tilt_pitch = merged_minus_tilt_pitch
            info_msg.merged_minus_tilt_yaw = merged_minus_tilt_yaw

            # Divergence checking. TODO: make it more efficient
            self.merged_minus_tilt_roll_vector.append(abs(merged_minus_tilt_roll))
            self.merged_minus_tilt_pitch_vector.append(abs(merged_minus_tilt_pitch))
            self.merged_minus_tilt_yaw_vector.append(abs(merged_minus_tilt_yaw))
            while len(self.merged_minus_tilt_roll_vector) > self.filter_divergence_buffer_size:
                self.merged_minus_tilt_roll_vector.pop(0)
                self.merged_minus_tilt_pitch_vector.pop(0)
                self.merged_minus_tilt_yaw_vector.pop(0)
            if len(self.merged_minus_tilt_roll_vector) == self.filter_divergence_buffer_size:
                self.divergence_factor = 1.0
                apply_mean_filter = False
                if (min(self.merged_minus_tilt_roll_vector) > 0.3):
                    self.divergence_factor = 0.8
                    apply_mean_filter = True
                    status += 'Roll has probably diverged. Trying to recover filter converge. '
                if (min(self.merged_minus_tilt_pitch_vector) > 0.3):
                    self.divergence_factor = 0.8
                    apply_mean_filter = True
                    status += 'Pitch has probably diverged. Trying to recover filter converge. '
                if (min(self.merged_minus_tilt_yaw_vector) > 0.3):
                    self.divergence_factor = 0.8
                    apply_mean_filter = True
                    status += 'Yaw has probably diverged. Trying to recover filter converge. '
                #if apply_mean_filter:
                #    status += 'Trying to remove bad elements from bias estimation vector. '
                #    remove_size = round(0.01 * len(self.bias_roll))
                #    if remove_size > 0:
                #        self.bias_roll = list(sort(self.bias_roll)[remove_size:-remove_size])
                #        self.bias_pitch = list(sort(self.bias_pitch)[remove_size:-remove_size])
                #        self.bias_yaw = list(sort(self.bias_yaw)[remove_size:-remove_size])
                #        status += 'Elements succesfully removed. '

            # Store status to the message
            if status == '':
                info_msg.status = 'Running fine.'
            else:
                info_msg.status = status.strip()

            # Publish info message
            info_msg.filter_elapsed_time = (rospy.Time.now() - self.past_time).to_sec()
            info_msg.header.stamp = rospy.Time.now()  # TODO: change by input time stamp
            info_msg.header.frame_id = self.frame_id
            self.info_pub.publish(info_msg)

            # Publish imu data message
            if self.bias_roll.size() > self.min_gyro_bias_buffer_size:
                msg = Imu()

                # aux = tf.transformations.quaternion_from_euler(self.merged_roll, self.merged_pitch, self.merged_yaw + self.declination)  # Added declination correction after all the filtering
                aux = tf.transformations.quaternion_from_euler(self.merged_roll, self.merged_pitch, self.merged_yaw)

                msg.orientation.x = aux[0]
                msg.orientation.y = aux[1]
                msg.orientation.z = aux[2]
                msg.orientation.w = aux[3]
                msg.orientation_covariance = [self.angle_covariance, 0.0, 0.0,
                                              0.0, self.angle_covariance, 0.0,
                                              0.0, 0.0, self.angle_covariance]

                msg.angular_velocity.x = corrected_angular_velocity_x
                msg.angular_velocity.y = corrected_angular_velocity_y
                msg.angular_velocity.z = corrected_angular_velocity_z
                msg.angular_velocity_covariance = [self.angular_velocity_covariance, 0.0, 0.0,
                                                   0.0, self.angular_velocity_covariance, 0.0,
                                                   0.0, 0.0, self.angular_velocity_covariance]

                msg.linear_acceleration.x = self.ax  # TODO: compute acceleration correction from imu position
                msg.linear_acceleration.y = self.ay
                msg.linear_acceleration.z = self.az
                msg.linear_acceleration_covariance = [self.acceleration_covariance, 0.0, 0.0,
                                                      0.0, self.acceleration_covariance, 0.0,
                                                      0.0, 0.0, self.acceleration_covariance]

                # Publish ros imu message
                msg.header.stamp = rospy.Time.now()  # TODO: change by input time stamp
                msg.header.frame_id = self.frame_id
                self.pub.publish(msg)


    def stop_magnetometer_calibration(self, req):
        if self.magnetometer_calibration_flag == 0:
            rospy.logwarn("%s: already stopped", self.name)
        else:
            rospy.loginfo("%s: stopping calibration", self.name)
            self.magnetometer_calibration_flag = 0
        return EmptyResponse()


    def calibrate_magnetometer(self, req):
        """ This service automatically calibrates the magnetometer """
        if self.magnetometer_calibration_flag != 0:
            rospy.logwarn("%s: calibration already running", self.name)
            return EmptyResponse()
        rospy.loginfo("%s: starting calibration", self.name)

        # Erase magnetometer calibration data
        self.ax_calib_vector = []
        self.ay_calib_vector = []
        self.az_calib_vector = []
        self.mx_calib_vector = []
        self.my_calib_vector = []
        self.mz_calib_vector = []
        self.filtered_tilt_corrected_mx_calib_vector = []
        self.filtered_tilt_corrected_my_calib_vector = []
        self.filtered_tilt_corrected_mz_calib_vector = []

        # Set flag
        self.magnetometer_calibration_flag = 1

        # Init message
        msg = BodyVelocityReq()
        msg.goal.priority = 30
        msg.goal.requester = self.name
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.disable_axis.x = True
        msg.disable_axis.y = True
        msg.disable_axis.z = True
        msg.disable_axis.roll = True
        msg.disable_axis.pitch = True

        # Start finite state machine
        r = rospy.Rate(self.rate)
        counter = 0
        elapsed_seconds = 0.0
        while elapsed_seconds < 255.0 and not rospy.is_shutdown() and self.magnetometer_calibration_flag == 1:
            msg.twist.angular.z = 0.0
            msg.disable_axis.yaw = True

            # From 0.0 to 5.0 seconds
            if elapsed_seconds >= 0.0 and elapsed_seconds < 5.0:
                rospy.loginfo("%s: spent %s/255.0 seconds. Starting", self.name, str(elapsed_seconds))

            # From 5.0 to 65.0 seconds
            if elapsed_seconds >= 5.0 and elapsed_seconds < 65.0:
                msg.twist.angular.z = 0.1
                msg.disable_axis.yaw = False
                rospy.loginfo("%s: spent %s/255.0 seconds. Turning right for 60.0 seconds, slow", self.name, str(elapsed_seconds))

            # From 65.0 to 125.0 seconds
            if elapsed_seconds >= 65.0 and elapsed_seconds < 125.0:
                msg.twist.angular.z = 0.2
                msg.disable_axis.yaw = False
                rospy.loginfo("%s: spent %s/255.0 seconds. Turning right for 60.0 seconds, fast", self.name, str(elapsed_seconds))

            # From 125.0 to 130.0 seconds
            if elapsed_seconds >= 125.0 and elapsed_seconds < 130.0:
                rospy.loginfo("%s: spent %s/255.0 seconds. Changing direction", self.name, str(elapsed_seconds))

            # From 130.0 to 190.0 seconds
            if elapsed_seconds >= 130.0 and elapsed_seconds < 190.0:
                msg.twist.angular.z = -0.1
                msg.disable_axis.yaw = False
                rospy.loginfo("%s: spent %s/255.0 seconds. Turning left for 60.0 seconds, slow", self.name, str(elapsed_seconds))

            # From 190.0 to 250.0 seconds
            if elapsed_seconds >= 190.0 and elapsed_seconds < 250.0:
                msg.twist.angular.z = -0.2
                msg.disable_axis.yaw = False
                rospy.loginfo("%s: spent %s/255.0 seconds. Turning left for 60.0 seconds, fast", self.name, str(elapsed_seconds))

            # From 250.0 to 255.0 seconds
            if elapsed_seconds >= 250.0 and elapsed_seconds < 255.0:
                rospy.loginfo("%s: spent %s/255.0 seconds. Slowing down", self.name, str(elapsed_seconds))

            # Publish
            msg.header.stamp = rospy.Time().now()
            self.pub_body_velocity_req.publish(msg)

            # Increment counter and compute elapsed seconds
            counter += 1
            elapsed_seconds = float(counter) / float(self.rate)

            # Sleep
            r.sleep()

        if self.magnetometer_calibration_flag == 1:
            rospy.loginfo("%s: computing calibration", self.name)

            # No more input data
            self.magnetometer_calibration_flag = 0

            # Verify data
            self.verify_magnetometer_calibration_data()

            # Compute calibration
            self.magnetometer_calibration_2d()

            rospy.loginfo("%s: finished", self.name)
        else:
            rospy.loginfo("%s: calibration stopped", self.name)

        return EmptyResponse()


    def enable_magnetometer(self, req):
        """ This service is used to enable magnetometer """
        self.magnetometer_enabled = True
        rospy.loginfo("%s: magnetometer enabled", self.name)
        return EmptyResponse()


    def disable_magnetometer(self, req):
        """ This service is used to disable magnetometer """
        self.magnetometer_enabled = False
        rospy.loginfo("%s: magnetometer disabled", self.name)
        return EmptyResponse()


    def freeze_gyro_bias(self, req):
        """ This service is used to freeze bias computation """
        self.freeze_gyro_bias_flag = True
        rospy.loginfo("%s: gyro bias computation has been frozen", self.name)
        return EmptyResponse()


    def unfreeze_gyro_bias(self, req):
        """ This service is used to unfreeze bias computation """
        self.freeze_gyro_bias_flag = False
        rospy.loginfo("%s: gyro bias computation has been unfrozen", self.name)
        return EmptyResponse()


    def reload_configuration(self, req):
        """ This service is used to reload config from param server """
        self.get_config()
        rospy.loginfo("%s: reloading configuration", self.name)
        return EmptyResponse()


    def verify_magnetometer_calibration_data(self):
        """ Verify data """
        try:
            # Verify length
            if len(self.mx_calib_vector) < 600:
                rospy.logwarn("%s: not enough data to use in calibration", self.name)

            # Data from all quads are needed
            quad = [False] * 4
            it = 0
            while not all(quad) and it < len(self.mx_calib_vector):
                if self.mx_calib_vector[it] > 0.0:
                    if self.my_calib_vector[it] > 0.0:
                        quad[0] = True
                    else:
                        quad[3] = True
                else:
                    if self.my_calib_vector[it] > 0.0:
                        quad[1] = True
                    else:
                        quad[2] = True
                it += 1
            if not all(quad):
                rospy.logwarn("%s: data not from all quad. in calibration", self.name)
        except:
            e = sys.exc_info()[1]
            rospy.logerr("%s: unexpected error in data verification: %s", self.name, str(e))


    def fit_ellipse(self, x_data, y_data):
        """ Fits an ellipse to x and y data """
        # TODO: optimize this method and improve variable names
        # Ellipse fit to ax^2+bxy+cy^2+dx+ey+f=0
        # Initialize some vars

        D1mat = matrix(zeros((len(x_data), 3)))
        D2mat = matrix(zeros((len(x_data), 3)))

        # Compute needed matrices from data buffers
        for it in range(len(x_data)):
            D1mat[it, 0] = x_data[it] * x_data[it]
            D1mat[it, 1] = x_data[it] * y_data[it]
            D1mat[it, 2] = y_data[it] * y_data[it]

            D2mat[it, 0] = x_data[it]
            D2mat[it, 1] = y_data[it]
            D2mat[it, 2] = 1.0

        S1mat = matrix(D1mat.T * D1mat)
        S2mat = matrix(D1mat.T * D2mat)
        S3mat = matrix(D2mat.T * D2mat)

        Tmat = matrix(-S3mat.I * S2mat.T)

        Mmat = matrix(S1mat + S2mat * Tmat)

        newMmat = matrix(zeros((3, 3)))
        newMmat[0, :] = Mmat[2, :] * 0.5
        newMmat[1, :] = -Mmat[1, :]
        newMmat[2, :] = Mmat[0, :] * 0.5

        evalues, evectors = linalg.eig(newMmat)

        cond0 = 4.0 * evectors[0, 0] * evectors[2, 0] - evectors[1, 0] * evectors[1, 0]
        if cond0 > 0:
            a1 = matrix(evectors[:, 0])
        cond1 = 4.0 * evectors[0, 1] * evectors[2, 1] - evectors[1, 1] * evectors[1, 1]
        if cond1 > 0:
            a1 = matrix(evectors[:,1])
        cond2 = 4.0 * evectors[0, 2] * evectors[2, 2] - evectors[1, 2] * evectors[1, 2]
        if cond2 > 0:
            a1 = matrix(evectors[:, 2])

        a2 = Tmat * a1

        return [a1[0, 0], a1[1, 0], a1[2, 0], a2[0, 0], a2[1, 0], a2[2, 0]]


    def upload_calibration_into_system(self, path_str, time_str):
        try:
            """ Print results to configuration file """
            file_str = 'imu_' + time_str + '.yaml'
            file_content = "# " + file_str + "\nimu_angle_estimator/soft_iron: [[" + str(self.imu_frame_S[0, 0]) + ", " + str(self.imu_frame_S[0, 1]) + ", " + str(self.imu_frame_S[0, 2]) + "], [" + str(self.imu_frame_S[1, 0]) + ", " + str(self.imu_frame_S[1, 1]) + ", " + str(self.imu_frame_S[1, 2]) + "], [" + str(self.imu_frame_S[2, 0]) + ", " + str(self.imu_frame_S[2, 1]) + ", " + str(self.imu_frame_S[2, 2]) + "]]\nimu_angle_estimator/hard_iron: [[" + str(self.imu_frame_H[0, 0]) + "], [" + str(self.imu_frame_H[1, 0]) + "], [" + str(self.imu_frame_H[2, 0]) + "]]\n"
            file_A = open(path_str + file_str, 'w+')
            file_A.write(file_content)
            file_A.close()
            file_B = open(path_str + 'imu_last_calibration.yaml', 'w+')
            file_B.write(file_content)
            file_B.close()

            """ Upload values in the node """
            self.soft_iron = self.imu_frame_S
            self.hard_iron = self.imu_frame_H

            """ Upload value in the param server """
            rospy.set_param(rospy.get_namespace() + 'imu_angle_estimator/soft_iron', [[float(self.imu_frame_S[0, 0]), float(self.imu_frame_S[0, 1]), float(self.imu_frame_S[0, 2])], [float(self.imu_frame_S[1, 0]), float(self.imu_frame_S[1, 1]), float(self.imu_frame_S[1, 2])], [float(self.imu_frame_S[2, 0]), float(self.imu_frame_S[2, 1]), float(self.imu_frame_S[2, 2])]])
            rospy.set_param(rospy.get_namespace() + 'imu_angle_estimator/hard_iron', [[float(self.imu_frame_H[0, 0])], [float(self.imu_frame_S[1, 0])], [float(self.imu_frame_S[2, 0])]])
        except:
            e = sys.exc_info()[1]
            rospy.logerr("%s: error printing configuration to file: %s", self.name, str(e))


    def print_magnetometer_results_to_image(self, path_str, time_str):
        try:
            """ Print results to image (plot) """
            # TODO: optimize this method and improve variable names
            _now = datetime.datetime.now()
            fig = plt.figure()
            fig.canvas.manager.set_window_title("IMU calibration: " + str(_now))
            sub_plot = fig.add_subplot(111, aspect='equal')

            # Raw
            aux_x = list(self.mx_calib_vector)
            aux_y = list(self.my_calib_vector)
            _pointsA = []
            while len(aux_x) > 0:
                _pointsA.append([aux_x.pop(0), aux_y.pop(0)])
            sub_plot.plot(*zip(*_pointsA), marker='o', color='r', ls='')  # Red -> raw

            # Filtered and tilt corrected
            aux_x = list(self.filtered_tilt_corrected_mx_calib_vector)
            aux_y = list(self.filtered_tilt_corrected_my_calib_vector)
            _pointsB = []
            while len(aux_x) > 0:
                _pointsB.append([aux_x.pop(0), aux_y.pop(0)])
            sub_plot.plot(*zip(*_pointsB), marker='o', color='g', ls='')  # Green -> filtered and tilt corrected

            # Magnetic correction
            aux_x = list(self.mx_calib_vector)
            aux_y = list(self.my_calib_vector)
            _pointsC = []
            while len(aux_x) > 0:
                mx = aux_x.pop(0)
                my = aux_y.pop(0)
                _pointsC.append([self.ned_frame_S[0, 0] * mx + self.ned_frame_S[0, 1] * my + self.ned_frame_H[0, 0], self.ned_frame_S[1, 0] * mx + self.ned_frame_S[1, 1] * my + self.ned_frame_H[1, 0]])
            sub_plot.plot(*zip(*_pointsC), marker='x', color='b', ls='')  # Blue -> magnetic correction

            # Whole correction
            aux_x = list(self.filtered_tilt_corrected_mx_calib_vector)
            aux_y = list(self.filtered_tilt_corrected_my_calib_vector)
            _pointsD = []
            while len(aux_x) > 0:
                mx = aux_x.pop(0)
                my = aux_y.pop(0)
                _pointsD.append([self.ned_frame_S[0, 0] * mx + self.ned_frame_S[0, 1] * my + self.ned_frame_H[0, 0], self.ned_frame_S[1, 0] * mx + self.ned_frame_S[1, 1] * my + self.ned_frame_H[1, 0]])
            sub_plot.plot(*zip(*_pointsD), marker='x', color='y', ls='')  # Yellow -> whole correction

            fig.suptitle('Magnetometer calibration\n(Red -> Raw, Green -> Filtered and tilt corrected, Blue -> Magnetic correction, Yellow -> Whole correction)', fontsize=10)
            sub_plot.set_xlabel('X axis [gauss]')
            sub_plot.set_ylabel('Y axis [gauss]')
            sub_plot.grid(True)
            plt.savefig(path_str + 'imu_' + time_str + '.png')
        except:
            e = sys.exc_info()[1]
            rospy.logerr("%s: error printing image to file: %s", self.name, str(e))


    def magnetometer_calibration_2d(self):
        """ Compute magnetometer calibration in 2d """
        # TODO: optimize this method and improve variable names
        try:
            # Leveling and tilt correction
            for it in range(len(self.ax_calib_vector)):
                if abs((self.ax_calib_vector[it] ** 2.0 + self.ay_calib_vector[it] ** 2.0 + self.az_calib_vector[it] ** 2.0) ** 0.5 - 9.80665) < self.acceleration_limit:  # Reject data with high acceleration
                    tilt_roll = math.atan2(-self.ay_calib_vector[it], -self.az_calib_vector[it])
                    #tilt_pitch = math.atan2(-self.ax_calib_vector[it], self.ay_calib_vector[it]*math.sin(tilt_roll) + self.az_calib_vector[it]*math.cos(tilt_roll))
                    den = -self.ay_calib_vector[it] * math.sin(tilt_roll) - self.az_calib_vector[it] * math.cos(tilt_roll)  # This is to have the pitch from pi/2 to -pi/2 (to force unique output)
                    if den != 0.0:
                        tilt_pitch = math.atan(self.ax_calib_vector[it] / den)
                    else:
                        if self.ax_calib_vector[it] == 0.0:
                            tilt_pitch = 0.0  # Not enough information
                        elif self.ax_calib_vector[it] < 0.0:
                            tilt_pitch = -math.pi * 0.5  # -pi/2
                        else:
                            tilt_pitch = math.pi * 0.5  # pi/2
                    rot1 = matrix([[math.cos(tilt_pitch), 0.0, math.sin(tilt_pitch)], [0.0, 1.0, 0.0], [-math.sin(tilt_pitch), 0.0, math.cos(tilt_pitch)]])
                    rot2 = matrix([[1.0, 0.0, 0.0], [0.0, math.cos(tilt_roll), -math.sin(tilt_roll)], [0.0, math.sin(tilt_roll), math.cos(tilt_roll)]])
                    rot = rot1 * rot2
                    corrected_m = rot * matrix([[self.mx_calib_vector[it]], [self.my_calib_vector[it]], [self.mz_calib_vector[it]]])
                    self.filtered_tilt_corrected_mx_calib_vector.append(corrected_m[0, 0])
                    self.filtered_tilt_corrected_my_calib_vector.append(corrected_m[1, 0])
                    self.filtered_tilt_corrected_mz_calib_vector.append(corrected_m[2, 0])

            # Check size of filtered data for calibration
            if len(self.filtered_tilt_corrected_mx_calib_vector) < 300:
                rospy.logwarn("%s: few data after filtering for calibration", self.name)

            # Fit initial ellipse
            try:
                [a, b, c, d, e, f] = self.fit_ellipse(self.filtered_tilt_corrected_mx_calib_vector, self.filtered_tilt_corrected_my_calib_vector)
            except:
                rospy.logerr("%s: error fitting initial ellipse", self.name)
                self.print_error_to_file("Error fitting initial ellipse")
                return

            # Center of the ellipse
            x_cent = (2.0 * c * d / b - e) / (b - 4.0 * a * c / b)
            y_cent = (d - 2.0 * a * e / b) / (4.0 * a * c / b - b)

            # Center ellipse
            x_calib_data = [x - x_cent for x in self.filtered_tilt_corrected_mx_calib_vector]
            y_calib_data = [y - y_cent for y in self.filtered_tilt_corrected_my_calib_vector]

            # Fit centered ellipse
            try:
                [a, b, c, d, e, f] = self.fit_ellipse(x_calib_data, y_calib_data)
            except:
                rospy.logerr("%s: error fitting centered ellipse", self.name)
                self.print_error_to_file("Error fitting centered ellipse")
                return

            # Angle of the major axis Q
            q = math.pow((math.pow(((c - a) / b), 2.0) + 1.0), 0.5) + (c - a) / b
            angle_1 = math.atan(q)
            angle_2 = math.atan(-1.0 / q)

            distance1 = math.pow((abs(-f / (a + b * math.tan(angle_1) + c * math.pow(math.tan(angle_1), 2.0))) + abs(-f / (a / math.pow(math.tan(angle_1), 2.0) + b / math.tan(angle_1) + c))), 0.5)
            distance2 = math.pow((abs(-f / (a + b * math.tan(angle_2) + c * math.pow(math.tan(angle_2), 2.0))) + abs(-f / (a / math.pow(math.tan(angle_2), 2.0) + b / math.tan(angle_2) + c))), 0.5)

            if distance1 > distance2:
                Q = angle_1
                scale_factor = distance1 / distance2
            else:
                Q = angle_2
                scale_factor = distance2 / distance1

            # Compose matrices
            ROT = matrix(eye(2))
            ROT[0, 0] = math.cos(Q)
            ROT[0, 1] = -math.sin(Q)
            ROT[1, 0] = math.sin(Q)
            ROT[1, 1] = math.cos(Q)

            preS = ROT * matrix([[1, 0], [0, scale_factor]]) * ROT.T
            S = matrix(eye(3))
            S[0, 0] = preS[0, 0]
            S[0, 1] = preS[0, 1]
            S[1, 0] = preS[1, 0]
            S[1, 1] = preS[1, 1]

            preH = matrix(zeros((3, 1)))
            preH[0, 0] = -x_cent
            preH[1, 0] = -y_cent

            self.ned_frame_S = S
            self.ned_frame_H = S * preH

            self.imu_frame_S = self.imu_rotation.T * self.ned_frame_S * self.imu_rotation
            self.imu_frame_H = self.imu_rotation.T * self.ned_frame_H

            rospack = rospkg.RosPack()
            path_str = rospack.get_path('cola2_' + str(rospy.get_namespace()).replace('/', '')) + '/calibrations/'
            time_str = str(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))
            self.upload_calibration_into_system(path_str, time_str)
            self.print_magnetometer_results_to_image(path_str, time_str)
        except:
            e = sys.exc_info()[1]
            rospy.logerr("%s: error in 2D magnetometer calibration: %s", self.name, str(e))


    def get_config(self):
        """ Get config from param server """
        valid_config = True

        if rospy.has_param("imu_angle_estimator/merging_filter_coefficient"):
            self.merging_filter_coefficient = rospy.get_param("imu_angle_estimator/merging_filter_coefficient")
        else:
            rospy.logfatal("%s: invalid parameters for imu_angle_estimator/merging_filter_coefficient in param server!", self.name)
            valid_config = False

        if rospy.has_param("imu_angle_estimator/max_gyro_bias_buffer_size"):
            self.max_gyro_bias_buffer_size = rospy.get_param("imu_angle_estimator/max_gyro_bias_buffer_size")
        else:
            rospy.logfatal("%s: invalid parameters for imu_angle_estimator/max_gyro_bias_buffer_size in param server!", self.name)
            valid_config = False

        if rospy.has_param("imu_angle_estimator/frame_id"):
            self.frame_id = rospy.get_namespace() + rospy.get_param("imu_angle_estimator/frame_id")
        else:
            rospy.logfatal("%s: invalid parameters for imu_angle_estimator/frame_id in param server!", self.name)
            valid_config = False

        if rospy.has_param("imu_angle_estimator/soft_iron"):
            self.soft_iron = matrix(rospy.get_param("imu_angle_estimator/soft_iron")).reshape(3, 3)
        else:
            rospy.logwarn("%s: invalid parameters for imu_angle_estimator/soft_iron in param server! Setting them to default.", self.name)
            self.soft_iron = matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

        if rospy.has_param("imu_angle_estimator/hard_iron"):
            self.hard_iron = matrix(rospy.get_param("imu_angle_estimator/hard_iron")).reshape(3, 1)
        else:
            rospy.logwarn("%s: invalid parameters for imu_angle_estimator/hard_iron in param server! Setting them to default.", self.name)
            self.hard_iron = matrix([[0.0], [0.0], [0.0]])

        if not valid_config:
            rospy.logfatal("%s: shutdown due to invalid config parameters!", self.name)
            exit(0)

        # Wait for TF
        while not rospy.is_shutdown():
            try:
                ok, xyz, rpy = self.tfh.get_transform(rospy.get_namespace() + "imu")
                if not ok:
                    raise Exception('Transform handler returned not ok')
                imu_rot = PyKDL.Rotation.RPY(rpy[0], rpy[1], rpy[2])
                self.imu_rotation = matrix([[imu_rot[0, 0], imu_rot[0, 1], imu_rot[0, 2]],
                                            [imu_rot[1, 0], imu_rot[1, 1], imu_rot[1, 2]],
                                            [imu_rot[2, 0], imu_rot[2, 1], imu_rot[2, 2]]])
                break
            except:
                rospy.loginfo("%s: waiting for %simu tf", self.name, rospy.get_namespace())
                rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        rospy.init_node('imu_angle_estimator')
        angle_estimator = IMU_ANGLE_ESTIMATOR(rospy.get_name())

        # Create the message filters
        imu_sub = message_filters.Subscriber('imu', Imu)
        mag_sub = message_filters.Subscriber('mag', MagneticField)
        ts = message_filters.TimeSynchronizer([imu_sub, mag_sub], 10)
        ts.registerCallback(angle_estimator.new_data_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass