#!/usr/bin/env python
"""
Copyright (c) 2014, 'CIRS - Universitat de Girona'
All rights reserved.
Software is provided WITHOUT WARRANTY and software authors and license owner
'CIRS - Universitat de Girona' cannot be held liable for any damages. You may
use and modify this software for PRIVATE USE. REDISTRIBUTION in source and
binary forms, with or without modification, are FORBIDDEN. You may NOT use the
names, logos, or trademarks of contributors.
"""


#
#  imu_angle_estimator.py
#
#  @date: 14/10/2013
#


import roslib
import rospy

from ned_tools import utils, utils_ros

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
import message_filters


class IMU_ANGLE_ESTIMATOR:
    """ This is the main class of the imu_angle_estimator.py node, which
        is able to compute imu output from mag, acc and gyros. It is also able
        to compute soft and hard iron calibrations and corrections """

    def __init__(self, name):
        """ Init the class """
        self.name = name

        # Init vars
        self.past_time = rospy.Time.now()  # Used to compute an euler derivative
        self.magnetometer_enabled = True
        self.freeze_gyro_bias_flag = False
        self.filter_init = False

        self.past_tilt_roll = 0.0  # This is used to compute an euler derivative of the tilt corrected angles to get euler rates
        self.past_tilt_pitch = 0.0
        self.past_tilt_yaw = 0.0

        self.bias_roll = []  # This is used to compute a mean filter for the angular velocities bias
        self.bias_pitch = []
        self.bias_yaw = []

        self.merged_minus_tilt_roll_vector = []  # This is used to check divergence
        self.merged_minus_tilt_pitch_vector = []
        self.merged_minus_tilt_yaw_vector = []
        self.divergence_factor = 1.0

        self.merged_roll = 0.0  # This is used when filtering
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

        # tf listener
        self.listener = tf.TransformListener()

        # Get config
        self.get_config()

        # Create publisher
        self.pub = rospy.Publisher(self.pub_topic,Imu,queue_size = 2)



        # Create services
        self.start_magnetometer_calibration_service = rospy.Service(
                         'imu_angle_estimator/start_magnetometer_calibration_service',
                         Empty, self.start_magnetometer_calibration)
        self.compute_magnetometer_calibration_service = rospy.Service(
                         'imu_angle_estimator/compute_magnetometer_calibration_service',
                         Empty, self.compute_magnetometer_calibration)
        self.freeze_gyro_bias_srv = rospy.Service(
                         'imu_angle_estimator/freeze_gyro_bias',
                         Empty, self.freeze_gyro_bias)
        self.unfreeze_gyro_bias_srv = rospy.Service(
                         'imu_angle_estimator/unfreeze_gyro_bias',
                         Empty, self.unfreeze_gyro_bias)

        # Initialized
        rospy.loginfo("[%s]: initialized", self.name)


    def imu_cb(self, imu_msg, mag_msg):
        """ This is the callback of the imu and mag messages that come from the
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
        # mag_vector = matrix([[mag_calib[0, 0]],
        #                      [mag_calib[1, 0]],
        #                      [mag_calib[2, 0]]])
        # mag_vector = self.imu_rotation * mag_vector
        mag_vector = self.imu_rotation * mag_calib

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
            e = sys.exc_info()[0]
            rospy.logerr("[%s]: unexpected error: %s", self.name, str(e))

        # Store data for calibration
        if self.magnetometer_calibration_flag == 1:
            # # Rotate magnetometer. No calibration here
            # mag_vector = matrix([[imu_message.MX],
            #                      [imu_message.MY],
            #                      [imu_message.MZ]])
            # mag_vector = self.imu_rotation * mag_vector

            # Store data to self
            self.ax_calib_vector.append(acc_vector[0, 0])
            self.ay_calib_vector.append(acc_vector[1, 0])
            self.az_calib_vector.append(acc_vector[2, 0])
            self.mx_calib_vector.append(mag_vector[0, 0])
            self.my_calib_vector.append(mag_vector[1, 0])
            self.mz_calib_vector.append(mag_vector[2, 0])


    def compute(self):
        """ Make an iteration """
        # Initialize and add some info to the string message
        status = ''
        if not self.magnetometer_enabled:
            status += 'Magnetometer has been disabled. '
        if self.freeze_gyro_bias_flag:
            status += 'Gyro bias computation has been frozen. '

        # Get period
        new_time = rospy.Time.now()
        real_period = (new_time - self.past_time).to_sec()
        self.past_time = new_time

        if real_period < 0.07:
            status += 'Period too small: ' + str(real_period) + ' seconds. Clipping at 0.07 seconds. '
            real_period = 0.07
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

        # Euler rates from tilt angles
        rate_tilt_roll = utils.wrapAngle(tilt_roll - self.past_tilt_roll) / real_period  # Simple Euler derivative
        rate_tilt_pitch = utils.wrapAngle(tilt_pitch - self.past_tilt_pitch) / real_period
        rate_tilt_yaw = utils.wrapAngle(tilt_yaw - self.past_tilt_yaw) / real_period
        self.past_tilt_roll = tilt_roll
        self.past_tilt_pitch = tilt_pitch
        self.past_tilt_yaw = tilt_yaw

        # Compute angular velocities from euler rates
        tilt_angular_velocity_x = rate_tilt_roll - rate_tilt_yaw * math.sin(tilt_pitch)
        tilt_angular_velocity_y = rate_tilt_pitch * math.cos(tilt_roll) + rate_tilt_yaw * math.cos(tilt_pitch) * math.sin(tilt_roll)
        tilt_angular_velocity_z = rate_tilt_yaw * math.cos(tilt_roll) * math.cos(tilt_pitch) - rate_tilt_pitch * math.sin(tilt_roll)

        # Euler rate bias, averaging filter and corrected euler rates
        bias_roll_from_this_iteration = self.gx - tilt_angular_velocity_x
        bias_pitch_from_this_iteration = self.gy - tilt_angular_velocity_y
        bias_yaw_from_this_iteration = self.gz - tilt_angular_velocity_z

        # Add new data to bias estimation buffers
        if self.magnetometer_enabled and not self.freeze_gyro_bias_flag:
            self.bias_roll.append(bias_roll_from_this_iteration)
            self.bias_pitch.append(bias_pitch_from_this_iteration)
            self.bias_yaw.append(bias_yaw_from_this_iteration)

        # Keep size of the buffers from infinite growing
        while len(self.bias_roll) > self.max_gyro_bias_buffer_size:
            self.bias_roll.pop(0)
            self.bias_pitch.pop(0)
            self.bias_yaw.pop(0)

        # Check size
        if len(self.bias_roll) < self.max_gyro_bias_buffer_size:
            status += 'Gyro bias buffers not full yet. '

        # Compute bias as a mean of bias buffers. Discarding some data is allowed
        if len(self.bias_roll) > 0:
            bias_roll = mean(self.bias_roll)
            bias_pitch = mean(self.bias_pitch)
            bias_yaw = mean(self.bias_yaw)
        else:
            bias_roll = 0.0
            bias_pitch = 0.0
            bias_yaw = 0.0


        # Compute corrected angular velocities
        corrected_angular_velocity_x = self.gx - bias_roll
        corrected_angular_velocity_y = self.gy - bias_pitch
        corrected_angular_velocity_z = self.gz - bias_yaw

        # Filter and merge data with complementary filter instead of an EKF
        if not self.filter_init:
            self.merged_roll = tilt_roll
            self.merged_pitch = tilt_pitch
            self.merged_yaw = tilt_yaw
            self.filter_init = True
        else:
            # Euler rates from angular velocities and tilt
            corrected_rate_roll_from_tilt = corrected_angular_velocity_x + corrected_angular_velocity_y * math.sin(tilt_roll) * math.tan(tilt_pitch) + corrected_angular_velocity_z * math.cos(tilt_roll) * math.tan(tilt_pitch)
            corrected_rate_pitch_from_tilt = corrected_angular_velocity_y * math.cos(tilt_roll) - corrected_angular_velocity_z * math.sin(tilt_roll)
            if math.cos(tilt_pitch) != 0.0:
                corrected_rate_yaw_from_tilt = corrected_angular_velocity_y * math.sin(tilt_roll) / math.cos(tilt_pitch) + corrected_angular_velocity_z * math.cos(tilt_roll) / math.cos(tilt_pitch)
            else:
                corrected_rate_yaw_from_tilt = 0.0

            # Euler rates from an iterative process
            limit_iterations = 20
            tolerance = 1e-8
            iteration_roll = tilt_roll
            iteration_pitch = tilt_pitch
            iteration_yaw = tilt_yaw
            i = 0
            found = False
            while (i < limit_iterations) and not found:
                i += 1
                corrected_rate_roll = corrected_angular_velocity_x + corrected_angular_velocity_y * math.sin(iteration_roll) * math.tan(iteration_pitch) + corrected_angular_velocity_z * math.cos(iteration_roll) * math.tan(iteration_pitch)
                corrected_rate_pitch = corrected_angular_velocity_y * math.cos(iteration_roll) - corrected_angular_velocity_z * math.sin(iteration_roll)
                if math.cos(iteration_pitch) != 0.0:
                    corrected_rate_yaw = corrected_angular_velocity_y * math.sin(iteration_roll) / math.cos(iteration_pitch) + corrected_angular_velocity_z * math.cos(iteration_roll) / math.cos(iteration_pitch)
                else:
                    corrected_rate_yaw = 0.0
                new_iteration_roll = utils.wrapAngle(self.merged_roll + corrected_rate_roll * real_period)
                new_iteration_pitch = utils.wrapAngle(self.merged_pitch + corrected_rate_pitch * real_period)
                new_iteration_yaw = utils.wrapAngle(self.merged_yaw + corrected_rate_yaw * real_period)
                if (abs(new_iteration_roll - iteration_roll) < tolerance) and (abs(new_iteration_pitch - iteration_pitch) < tolerance) and (abs(new_iteration_yaw - iteration_yaw) < tolerance):
                    found = True
                else:
                    iteration_roll = new_iteration_roll
                    iteration_pitch = new_iteration_pitch
                    iteration_yaw = new_iteration_yaw
            if i == limit_iterations:
                corrected_rate_roll = corrected_rate_roll_from_tilt
                corrected_rate_pitch = corrected_rate_pitch_from_tilt
                corrected_rate_yaw = corrected_rate_yaw_from_tilt
                status += 'Maximum number of iterations reached. '


            # If there are huge accelerations or rotations, use more gyros
            fade_factor = 0.7 + 0.3 * (float(len(self.bias_roll)) / float(self.min_gyro_bias_buffer_size)) ** 0.5
            if fade_factor > 1.0:
                fade_factor = 1.0
            merging_filter_coefficient = self.merging_filter_coefficient * fade_factor * self.divergence_factor
            if abs((self.ax ** 2.0 + self.ay ** 2.0 + self.az ** 2.0) ** 0.5 - 9.80665) > self.acceleration_limit:
                merging_filter_coefficient = 1.0 - self.filter_reduction_factor * (1.0 - merging_filter_coefficient)
            if abs(corrected_rate_roll) + abs(corrected_rate_pitch) + abs(corrected_rate_yaw) > self.rates_limit:
                merging_filter_coefficient = 1.0 - self.filter_reduction_factor * (1.0 - merging_filter_coefficient)
            if not self.magnetometer_enabled:  # If magnetometer is disabled, use only gyros
                merging_filter_coefficient = 1.0


            # Compute gyro innovation
            gyro_roll_innovation = corrected_rate_roll * real_period
            gyro_pitch_innovation = corrected_rate_pitch * real_period
            gyro_yaw_innovation = corrected_rate_yaw * real_period

            # Compute tilt innovation
            tilt_roll_innovation = utils.wrapAngle(tilt_roll - self.merged_roll)
            tilt_pitch_innovation = utils.wrapAngle(tilt_pitch - self.merged_pitch)
            tilt_yaw_innovation = utils.wrapAngle(tilt_yaw - self.merged_yaw)

            # Compute innovation
            roll_innovation = merging_filter_coefficient * gyro_roll_innovation + (1.0 - merging_filter_coefficient) * tilt_roll_innovation
            pitch_innovation = merging_filter_coefficient * gyro_pitch_innovation + (1.0 - merging_filter_coefficient) * tilt_pitch_innovation
            yaw_innovation = merging_filter_coefficient * gyro_yaw_innovation + (1.0 - merging_filter_coefficient) * tilt_yaw_innovation

            # Compute new state (merged_xxxx may not follow the -180 to 180 for roll, -90 to 90 for pitch and -180 to 180 for yaw convention, but as the output will be a quaternion, this doesn't matter)
            self.merged_roll = utils.wrapAngle(self.merged_roll + roll_innovation)
            self.merged_pitch = utils.wrapAngle(self.merged_pitch + pitch_innovation)
            self.merged_yaw = utils.wrapAngle(self.merged_yaw + yaw_innovation)
            merged_minus_tilt_roll = utils.wrapAngle(self.merged_roll - tilt_roll)
            merged_minus_tilt_pitch = utils.wrapAngle(self.merged_pitch - tilt_pitch)
            merged_minus_tilt_yaw = utils.wrapAngle(self.merged_yaw - tilt_yaw)


            # Divergence checking. TODO: make it more efficient
            self.merged_minus_tilt_roll_vector.append(merged_minus_tilt_roll)
            self.merged_minus_tilt_pitch_vector.append(merged_minus_tilt_pitch)
            self.merged_minus_tilt_yaw_vector.append(merged_minus_tilt_yaw)
            while len(self.merged_minus_tilt_roll_vector) > self.filter_divergence_buffer_size:
                self.merged_minus_tilt_roll_vector.pop(0)
                self.merged_minus_tilt_pitch_vector.pop(0)
                self.merged_minus_tilt_yaw_vector.pop(0)
            if len(self.merged_minus_tilt_roll_vector) == self.filter_divergence_buffer_size:
                self.divergence_factor = 1.0
                if (not (any([x < 0.0 for x in self.merged_minus_tilt_roll_vector]) and any([x >= 0.0 for x in self.merged_minus_tilt_roll_vector]))) or (min([abs(x) for x in self.merged_minus_tilt_roll_vector]) > 0.3):
                    self.divergence_factor = 0.8
                    status += 'Roll has probably diverged. Trying to recover filter converge. '
                if (not (any([x < 0.0 for x in self.merged_minus_tilt_pitch_vector]) and any([x >= 0.0 for x in self.merged_minus_tilt_pitch_vector]))) or (min([abs(x) for x in self.merged_minus_tilt_pitch_vector]) > 0.3):
                    self.divergence_factor = 0.8
                    status += 'Pitch has probably diverged. Trying to recover filter converge. '
                if (not (any([x < 0.0 for x in self.merged_minus_tilt_yaw_vector]) and any([x >= 0.0 for x in self.merged_minus_tilt_yaw_vector]))) or (min([abs(x) for x in self.merged_minus_tilt_yaw_vector]) > 0.3):
                    self.divergence_factor = 0.8
                    status += 'Yaw has probably diverged. Trying to recover filter converge. '

            # Publish imu data message
            if len(self.bias_roll) > self.min_gyro_bias_buffer_size:
                msg = Imu()
                aux = tf.transformations.quaternion_from_euler(self.merged_roll, self.merged_pitch, self.merged_yaw + self.declination)  # Added declination correction after all the filtering
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
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.robot_frame_id  # Imu data is corrected to be referred to the robot frame id
                self.pub.publish(msg)


    def start_magnetometer_calibration(self, req):
        """ Start magnetometer calibration service callback """
        if self.magnetometer_calibration_flag == 0:
            self.magnetometer_calibration_flag = 1
            rospy.loginfo("[%s]: starting calibration", self.name)
        else:
            rospy.logwarn("[%s]: calibration already running", self.name)
        return EmptyResponse()


    def compute_magnetometer_calibration(self, req):
        """ Compute magnetometer calibration service callback """
        if self.magnetometer_calibration_flag == 1:
            self.magnetometer_calibration_flag = 2  # No more input data
            rospy.loginfo("[%s]: computing calibration", self.name)

            # Verify data
            self.verify_magnetometer_calibration_data()

            # Compute calibration
            self.magnetometer_calibration_2d()

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

            # Set flag back to 0
            self.magnetometer_calibration_flag = 0
        else:
            rospy.logerr("[%s]: start a calibration before attempting to compute", self.name)
        return EmptyResponse()


    def enable_magnetometer(self, req):
        """ This service is used to enable magnetometer """
        self.magnetometer_enabled = True
        rospy.loginfo("[%s]: magnetometer enabled", self.name)
        return EmptyResponse()


    def disable_magnetometer(self, req):
        """ This service is used to disable magnetometer """
        self.magnetometer_enabled = False
        rospy.loginfo("[%s]: magnetometer disabled", self.name)
        return EmptyResponse()


    def freeze_gyro_bias(self, req):
        """ This service is used to freeze bias computation """
        self.freeze_gyro_bias_flag = True
        rospy.loginfo("[%s]: gyro bias computation has been frozen", self.name)
        return EmptyResponse()


    def unfreeze_gyro_bias(self, req):
        """ This service is used to unfreeze bias computation """
        self.freeze_gyro_bias_flag = False
        rospy.loginfo("[%s]: gyro bias computation has been unfrozen", self.name)
        return EmptyResponse()


    def reload_configuration(self, req):
        """ This service is used to reload config from param server """
        self.get_config()
        rospy.loginfo("[%s]: reloading configuration", self.name)
        return EmptyResponse()


    def verify_magnetometer_calibration_data(self):
        """ Verify data """
        try:
            # Verify length
            if len(self.mx_calib_vector) < 600:
                rospy.logwarn("[%s]: not enough data to use in calibration", self.name)

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
                rospy.logwarn("[%s]: data not from all quad. in calibration", self.name)
        except:
            rospy.logerr("[%s]: unexpected error in data verification", self.name)


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


    def print_magnetometer_results_to_file(self):
        # try:
        """ Print results to file """
        _file = open(self.calibration_file_path + '/imu_calibration.txt', 'w')
        _now = datetime.datetime.now()
        _header = "\n------------------------------------------------------------------------------------------\nMAGNETOMETER CALIBRATION, " + str(_now) + "\n\n"
        _mat_results = "Soft iron:\n" + str(self.imu_frame_S) + "\n\nHard iron:\n" + str(self.imu_frame_H) + "\n\n"
        _num_results = "imu_angle_estimator/soft_iron: [[" + str(self.imu_frame_S[0, 0]) + ", " + str(self.imu_frame_S[0, 1]) + ", " + str(self.imu_frame_S[0, 2]) + "], [" + str(self.imu_frame_S[1, 0]) + ", " + str(self.imu_frame_S[1, 1]) + ", " + str(self.imu_frame_S[1, 2]) + "], [" + str(self.imu_frame_S[2, 0]) + ", " + str(self.imu_frame_S[2, 1]) + ", " + str(self.imu_frame_S[2, 2]) + "]]\nimu_angle_estimator/hard_iron: [[" + str(self.imu_frame_H[0, 0]) + "], [" + str(self.imu_frame_H[1, 0]) + "], [" + str(self.imu_frame_H[2, 0]) + "]]\n\n\n"
        _data = "Data:\nMX:\n" + str(self.mx_calib_vector) + "\n\nMY:\n" + str(self.my_calib_vector) + "\n\nMZ:\n" + str(self.mz_calib_vector) + "\n\nAX:\n" + str(self.ax_calib_vector) + "\n\nAY:\n" + str(self.ay_calib_vector) + "\n\nAZ:\n" + str(self.az_calib_vector) + "\n\nFILTERED_MX:\n" + str(self.filtered_tilt_corrected_mx_calib_vector) + "\n\nFILTERED_MY:\n" + str(self.filtered_tilt_corrected_my_calib_vector) + "\n\nFILTERED_MZ:\n" + str(self.filtered_tilt_corrected_mz_calib_vector) + "\n\n"
        _string = _header + _mat_results + _num_results + _data
        _file.write(_string)
        _file.close()
        # except:
        #     rospy.logerr("[%s]: error printing results to file", self.name)


    def print_error_to_file(self, _error):
        try:
            """ Print error to file """
            _file = open(self.calibration_file_path + '/imu_calibration.txt', 'w')
            _now = datetime.datetime.now()
            _header = "\n------------------------------------------------------------------------------------------\nMAGNETOMETER CALIBRATION, " + str(_now) + "\n\n"
            _data = "Data:\nMX:\n" + str(self.mx_calib_vector) + "\n\nMY:\n" + str(self.my_calib_vector) + "\n\nMZ:\n" + str(self.mz_calib_vector) + "\n\nAX:\n" + str(self.ax_calib_vector) + "\n\nAY:\n" + str(self.ay_calib_vector) + "\n\nAZ:\n" + str(self.az_calib_vector) + "\n\nFILTERED_MX:\n" + str(self.filtered_tilt_corrected_mx_calib_vector) + "\n\nFILTERED_MY:\n" + str(self.filtered_tilt_corrected_my_calib_vector) + "\n\nFILTERED_MZ:\n" + str(self.filtered_tilt_corrected_mz_calib_vector) + "\n\n"
            _string = _header + "Error:\n" + _error + "\n\n" + _data
            _file.write(_string)
            _file.close()
        except:
            rospy.logerr("[%s]: error printing error to file", self.name)


    def print_magnetometer_results_to_image(self):
        try:
            """ Print results to image (plot) """
            # TODO: optimize this method and improve variable names
            _now = datetime.datetime.now()
            fig = plt.figure()
            fig.canvas.manager.set_window_title("IMU calibration: " + str(_now))
            sub_plot = fig.add_subplot(111, aspect='equal')

            # Without correction
            aux_x = list(self.mx_calib_vector)
            aux_y = list(self.my_calib_vector)
            _pointsA = []
            while len(aux_x) > 0:
                _pointsA.append([aux_x.pop(0), aux_y.pop(0)])
            sub_plot.plot(*zip(*_pointsA), marker='o', color='r', ls='')  # Red -> raw data

            # With tilt correction
            aux_x = list(self.filtered_tilt_corrected_mx_calib_vector)
            aux_y = list(self.filtered_tilt_corrected_my_calib_vector)
            _pointsB = []
            while len(aux_x) > 0:
                _pointsB.append([aux_x.pop(0), aux_y.pop(0)])
            sub_plot.plot(*zip(*_pointsB), marker='o', color='g', ls='')  # Green -> tilt correction

            # With whole correction
            aux_x = list(self.mx_calib_vector)
            aux_y = list(self.my_calib_vector)
            _pointsC = []
            while len(aux_x) > 0:
                mx = aux_x.pop(0)
                my = aux_y.pop(0)
                _pointsC.append([self.ned_frame_S[0, 0] * mx + self.ned_frame_S[0, 1] * my + self.ned_frame_H[0, 0], self.ned_frame_S[1, 0] * mx + self.ned_frame_S[1, 1] * my + self.ned_frame_H[1, 0]])
            sub_plot.plot(*zip(*_pointsC), marker='x', color='b', ls='')  # Blue -> whole correction
            sub_plot.set_title('Magnetometer calibration\n(Red -> Raw, Green -> Filtered and tilt corrected, Blue -> Whole correction)')
            sub_plot.set_xlabel('X axis [gauss]')
            sub_plot.set_ylabel('Y axis [gauss]')
            plt.savefig(self.calibration_file_path + '/imu_calibration_plot.png')
        except:
            rospy.logerr("[%s]: error printing image to file", self.name)


    def magnetometer_calibration_2d(self):
        """ Compute magnetometer calibration in 2d """
        # TODO: optimize this method and improve variable names
        # try:
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
            rospy.logwarn("[%s]: few data after filtering for calibration", self.name)

        # Fit initial ellipse
        try:
            [a, b, c, d, e, f] = self.fit_ellipse(self.filtered_tilt_corrected_mx_calib_vector, self.filtered_tilt_corrected_my_calib_vector)
        except:
            rospy.logerr("[%s]: error fitting initial ellipse", self.name)
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
            rospy.logerr("[%s]: error fitting centered ellipse", self.name)
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
        preH[0,0] = -x_cent
        preH[1,0] = -y_cent

        self.ned_frame_S = S
        self.ned_frame_H = S * preH

        self.imu_frame_S =  self.ned_frame_S
        self.imu_frame_H =  self.ned_frame_H

        # self.imu_frame_S = self.imu_rotation.T * self.ned_frame_S * self.imu_rotation
        # self.imu_frame_H = self.imu_rotation.T * self.ned_frame_H

        self.print_magnetometer_results_to_file()
        self.print_magnetometer_results_to_image()
        # except:
        #     rospy.logerr("[%s]: error in self.magnetometer_calibration_2d", self.name)
        #     self.print_error_to_file("Unexpected error in calibration")


    def get_config(self):
        """ Get config from param server """
        param_dict = {
                    'merging_filter_coefficient': 'imu_angle_estimator/merging_filter_coefficient',  
                    'max_gyro_bias_buffer_size': 'imu_angle_estimator/max_gyro_bias_buffer_size',  
                    'min_gyro_bias_buffer_size': 'imu_angle_estimator/min_gyro_bias_buffer_size',  
                    'filter_divergence_buffer_size': 'imu_angle_estimator/filter_divergence_buffer_size',  
                    'acceleration_limit': 'imu_angle_estimator/acceleration_limit',  
                    'rates_limit': 'imu_angle_estimator/rates_limit',  
                    'filter_reduction_factor': 'imu_angle_estimator/filter_reduction_factor',  
                    'pub_topic': 'imu_angle_estimator/pub_topic',  
                    'frame_id': 'imu_angle_estimator/frame_id',  
                    'robot_frame_id': 'frames/base_link',  
                    'angle_covariance': 'imu_angle_estimator/angle_covariance',  
                    'angular_velocity_covariance': 'imu_angle_estimator/angular_velocity_covariance',  
                    'acceleration_covariance': 'imu_angle_estimator/acceleration_covariance',  
                    'declination': 'imu_angle_estimator/declination',
                    'soft_iron': 'imu_angle_estimator/soft_iron',  
                    'hard_iron': 'imu_angle_estimator/hard_iron',
                    'calibration_file_path': 'imu_angle_estimator/calibration_file_path'}

        if not utils_ros.getRosParams(self, param_dict, self.name):
            rospy.logfatal("%s: shutdown due to invalid config parameters!", self.name)
            exit(0)  # TODO: find a better way

        try:
            self.listener.waitForTransform(self.robot_frame_id,
                                           self.frame_id,
                                           rospy.Time(),
                                           rospy.Duration(5.0))

            (trans, rot) = self.listener.lookupTransform(
                              self.robot_frame_id, self.frame_id, rospy.Time())
            euler = tf.transformations.euler_from_quaternion(rot)
            imu_rot = PyKDL.Rotation.RPY(euler[0], euler[1], euler[2])
            self.imu_rotation = matrix([[imu_rot[0, 0],imu_rot[0, 1],imu_rot[0, 2]],
                                        [imu_rot[1, 0],imu_rot[1, 1],imu_rot[1, 2]],
                                        [imu_rot[2, 0],imu_rot[2, 1],imu_rot[2, 2]]])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logfatal("[%s]: Impossible to transform from %s to %s", self.name, self.robot_frame_id, self.frame_id)
            valid_config = False


if __name__ == '__main__':
    try:

        rospy.init_node('imu_angle_estimator')
        angle_estimator = IMU_ANGLE_ESTIMATOR(rospy.get_name())
        imu_sub = message_filters.Subscriber('imu', Imu)
        mag_sub = message_filters.Subscriber('mag', MagneticField)
        ts = message_filters.TimeSynchronizer([imu_sub, mag_sub], 10)
        ts.registerCallback(angle_estimator.imu_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
