#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

import roslib
import rospy

from cola2_lib.diagnostic_helper import DiagnosticHelper
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from cola2_msgs.msg import VehicleStatus, CaptainStatus, TotalTime
from auv_msgs.msg import NavSts


class VehicleStatusParser:
    """ This node generates the VehicleStatus message from multiple sources."""

    def __init__(self, name):
        """ Init the class """
        self.name = name
        self.status = VehicleStatus()

        # Create Publishers
        self.pub_vehicle_sts = rospy.Publisher("/cola2_safety/vehicle_status",
                                               VehicleStatus,
                                               queue_size=1)

        # Subscriber
        rospy.Subscriber("/diagnostics_agg",
                         DiagnosticArray,
                         self.update_diagnostics,
                         queue_size=1)

        rospy.Subscriber("/cola2_control/captain_status",
                         CaptainStatus,
                         self.update_captain_status,
                         queue_size=1)

        rospy.Subscriber("/cola2_safety/total_time",
                         TotalTime,
                         self.update_timeout,
                         queue_size=1)

        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.update_nav_sts,
                         queue_size=1)

        rospy.Subscriber("/cola2_control/thrusters_enabled",
                         Bool,
                         self.update_thruster_status,
                         queue_size=1)

    def update_timeout(self, timeout):
        """Update timeout in vehicle status."""
        self.status.up_time = timeout.total_time

    def update_thruster_status(self, thruster_status):
        """Update timeout in vehicle status."""
        self.status.thrusters_enabled = thruster_status.data

    def update_captain_status(self, captain_status):
        """Update captain status information."""
        self.status.active_controller = captain_status.active_controller
        self.status.altitude_mode = captain_status.altitude_mode
        self.status.mission_active = captain_status.mission_active
        self.status.current_step = captain_status.current_step
        self.status.total_steps = captain_status.total_steps

    def update_nav_sts(self, nav):
        """Update navigation information."""
        self.status.latitude = nav.global_position.latitude
        self.status.longitude = nav.global_position.longitude
        self.status.heading = nav.orientation.yaw
        self.status.altitude = nav.altitude
        self.status.depth = nav.position.depth
        if nav.position.depth < 0.5:
            self.status.at_surface = True
        else:
            self.status.at_surface = False

    def update_diagnostics(self, diagnostics):
        """ Check diagnostics messages to fill vehicle status."""
        for status in diagnostics.status:

            # Get vehicle initialized status
            if __getDiagnostic__(status, '/navigation/ navigator'):
                if __getDiagnostic__(status,
                                     '/navigation/ navigator',
                                     'ekf_init',
                                     'False') == 'True':
                    self.status.vehicle_initialized = True
                else:
                    self.status.vehicle_initialized = False

            # Get battery charge and voltage
            if __getDiagnostic__(status, '/safety/ battery'):
                charge = float(__getDiagnostic__(status,
                                                 '/safety/ battery',
                                                 'charge',
                                                 100.0))
                voltage = float(__getDiagnostic__(status,
                                                 '/safety/ battery',
                                                 'voltage',
                                                 100.0))
                self.status.battery_charge = charge
                self.status.battery_voltage = voltage

            # Get IMU data age
            if __getDiagnostic__(status, '/navigation/ navigator'):
                imu_age = float(__getDiagnostic__(status,
                                                  '/navigation/ navigator',
                                                  'last_imu_data',
                                                  0.0))
                if abs(imu_age) > 10000.0:  # To avoid initialization problems
                    imu_age = 0.0
                self.status.imu_data_age = imu_age

            # Get depth data age
            if __getDiagnostic__(status, '/navigation/ navigator'):
                depth_age = float(__getDiagnostic__(status,
                                                    '/navigation/ navigator',
                                                    'last_depth_data',
                                                    0.0))
                if abs(depth_age) > 10000.0:  # To avoid initialization problems
                    depth_age = 0.0
                self.status.depth_data_age = depth_age

            # Get altitude data age
            if __getDiagnostic__(status, '/navigation/ navigator'):
                altitude_age = float(__getDiagnostic__(status,
                                                       '/navigation/ navigator',
                                                       'last_altitude_data',
                                                       0.0))
                if abs(altitude_age) > 10000.0:  # To avoid initialization problems
                    altitude_age = 0.0
                self.status.altitude_data_age = altitude_age

            # Get DVL data age
            if __getDiagnostic__(status, '/navigation/ navigator'):
                dvl_age = float(__getDiagnostic__(status,
                                                  '/navigation/ navigator',
                                                  'last_dvl_data',
                                                  0.0))
                if abs(dvl_age) > 10000.0:  # To avoid initialization problems
                    dvl_age = 0.0
                self.status.dvl_data_age = dvl_age

            # Get GPS data age
            if __getDiagnostic__(status, '/navigation/ navigator'):
                gps_age = float(__getDiagnostic__(status,
                                                  '/navigation/ navigator',
                                                  'last_gps_data',
                                                  0.0))
                if abs(gps_age) > 10000.0:  # To avoid initialization problems
                    gps_age = 0.0
                self.status.gps_data_age = gps_age

            # Get Navigation data age
            if __getDiagnostic__(status, '/safety/ up_time'):
                nav_age = float(__getDiagnostic__(status,
                                                  '/safety/ up_time',
                                                  'last_nav_data',
                                                  0.0))
                if abs(nav_age) > 10000.0:  # To avoid initialization problems
                    nav_age = 0.0
                self.status.navigation_data_age = nav_age

            # Get DVL valid data age
            if __getDiagnostic__(status, '/navigation/ teledyne_explorer_dvl'):
                dvl_age = float(__getDiagnostic__(status,
                                                  '/navigation/ teledyne_explorer_dvl',
                                                  'last_good_data',
                                                  0.0))
                if abs(dvl_age) > 100000.0:
                    dvl_age = 0.0
                self.status.dvl_valid_data_age = dvl_age

            # Get WIFI age
            if __getDiagnostic__(status, '/control/ teleoperation'):
                wifi_age = float(__getDiagnostic__(status,
                                                   '/control/ teleoperation',
                                                   'last_ack',
                                                   0.0))
                if abs(wifi_age) > 100000.0:
                    wifi_age = 0.0
                self.status.wifi_data_age = wifi_age

            # Get modem data age
            if __getDiagnostic__(status, '/safety/ evologics_modem'):
                modem_age = float(__getDiagnostic__(status,
                                                    '/safety/ evologics_modem',
                                                    'last_modem_data',
                                                    0.0))
                if abs(modem_age) > 100000.0:
                    modem_age = 0.0
                self.status.modem_data_age = modem_age

            # Temperature
            if __getDiagnostic__(status, '/safety/ battery') or \
                    __getDiagnostic__(status, '/safety/ computer_logger') or \
                    __getDiagnostic__(status, '/control/ s2_actuators'):
                temp_bat = float(__getDiagnostic__(status,
                                                   '/safety/ battery',
                                                   'max_cell_temperature',
                                                   0.0))
                temp_pc = float(__getDiagnostic__(status,
                                                  '/safety/ computer_logger',
                                                  'cpu_temperature',
                                                  0.0))
                temp_thr = float(__getDiagnostic__(status,
                                                   '/control/ s2_actuators',
                                                   'max_thruster_temperature',
                                                   0.0))
                self.status.internal_temperature = [temp_bat, temp_pc, temp_thr]

            # Water inside
            if __getDiagnostic__(status, '/safety/ s2_mon_control_board') or \
               __getDiagnostic__(status, '/control/ s2_actuators'):
                water_main = __getDiagnostic__(status,
                                               '/safety/ s2_mon_control_board',
                                               'water_inside',
                                               'False') == 'True'
                water_fins = __getDiagnostic__(status,
                                               '/control/ s2_actuators',
                                               'water_inside_fins',
                                               'False') == 'True'
                if water_main or water_fins:
                    self.status.water_detected = True
                else:
                    self.status.water_detected = False

        # Publish status message
        self.status.header.stamp = rospy.Time.now()
        self.status.header.frame_id = 'sparus2'
        self.pub_vehicle_sts.publish(self.status)


def __getDiagnostic__(status, name, key='none', default=0.0):
    if status.name == name:
        if key != 'none':
            return __getValue__(status.values, key, default)
        else:
            return True
    return False


def __getValue__(values, key, default):
    for pair in values:
        if pair.key == key:
            return pair.value
    return default


if __name__ == '__main__':
    try:
        rospy.init_node('vehicle_status')
        vehicle_status = VehicleStatusParser(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
