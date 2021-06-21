#!/usr/bin/env python
# Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
@@>This node subscribes to the aggregated diagnostics and publishes a vehicle status message.<@@
"""

import rospy
from std_msgs.msg import Bool
import numpy as np
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from cola2_msgs.msg import VehicleStatus, MissionStatus, CaptainStatus, NavSts
from std_msgs.msg import Int32
from cola2_lib.rosutils.diagnostic_helper import DiagnosticHelper
from cola2_lib.rosutils import param_loader
from cola2_xiroi.msg import PowerReading

class VehicleStatusParser:
    """
    This node subscribes to vehicle information published in multiple topics and concentrates it
    in a VehicleStatus message.
    The node is also in charge of publishing the last_nav_data diagnostics (since it cannot be checked from
    within the navigator).
    """

    def __init__(self, name):
        # Battery SoC and voltage mean
        self.soc_buffer = np.array([])
        self.volts_buffer = np.array([])
        self.first_time_soc = True
        self.first_time_voltage = True
        self.buffer_size = rospy.get_param("buffer_size", 50)

        self.name = name

        # Get config parameters
        self.get_config()

        self.status = VehicleStatus()

        # Set up diagnostics for last nav data
        self.diagnostic = DiagnosticHelper(self.name, "soft")
        self.init_navigator_check = False
        self.last_navigator_callback = rospy.Time.now().to_sec()



        namespace = rospy.get_namespace()
              # Create Publisher
        self.pub_vehicle_sts = rospy.Publisher(namespace + "vehicle_status", VehicleStatus, queue_size=1)

        # Initialize battery data
        self.status.battery_charge  = 100.0
        self.status.battery_voltage = 14  # Which value do I put? This should not be here...
        self.last_battery_charge  = rospy.Time.now()
        self.last_battery_voltage = rospy.Time.now()

        # Check time since last water inside data
        self.last_water_inside = rospy.Time.now()

        # Initialize temperature vector
        self.status.temperature = [-1000.0] * len(self.temperature_name)
        t = rospy.Time.now()
        self.last_temperature = [t] * len(self.temperature_name)

        # Initialize water vector
        self.water = [False] * len(self.diag_water)

        # Initialize virtual cage state
        self.status.inside_virtual_cage = True

        # Subscriber
        rospy.Subscriber(namespace + "diagnostics_agg",
                         DiagnosticArray,
                         self.update_diagnostics,
                         queue_size=1)

        rospy.Subscriber(namespace + "captain/mission_status",
                         MissionStatus,
                         self.update_mission_status,
                         queue_size=1)

        rospy.Subscriber(namespace + "captain/captain_status",
                         CaptainStatus,
                         self.update_captain_status,
                         queue_size=1)

        rospy.Subscriber(namespace + "cola2_watchdog/elapsed_time",
                         Int32,
                         self.update_timeout,
                         queue_size=1)

        rospy.Subscriber(namespace + "navigator/navigation",
                         NavSts,
                         self.update_nav_sts,
                         queue_size=1)

        rospy.Subscriber(namespace + "sensors/battery",
                         PowerReading,
                         self.update_battery_info,
                         queue_size=1)

    def update_battery_info(self, battery_msg):
        """Update timeout in vehicle status."""

        if(self.first_time_soc==True):
            for i in range(self.buffer_size):
                self.soc_buffer = np.append(self.soc_buffer,battery_msg.input_soc)
        self.first_time_soc = False

        #roll,rewrite new value and do the mean
        self.soc_buffer = np.roll(self.soc_buffer, -1)
        self.soc_buffer = np.insert(self.soc_buffer,self.buffer_size-1,battery_msg.input_soc)
        self.soc_buffer = np.delete(self.soc_buffer,self.buffer_size)
        self.status.battery_charge = np.mean(self.soc_buffer)

        if(self.first_time_voltage==True):
            for i in range(self.buffer_size):
                self.volts_buffer = np.append(self.volts_buffer,battery_msg.volts_read_value[0])
        self.first_time_voltage = False

        #roll,rewrite new value and do the mean
        self.volts_buffer = np.roll(self.volts_buffer, -1)
        self.volts_buffer = np.insert(self.volts_buffer,self.buffer_size-1,battery_msg.volts_read_value[0])
        self.volts_buffer = np.delete(self.volts_buffer,self.buffer_size)
        self.status.battery_voltage = np.mean(self.volts_buffer)


    def update_timeout(self, watchdog_msg):
        """Update timeout in vehicle status."""
        #self.status.up_time = watchdog_msg.elapsed_time
        self.status.elapsed_time = watchdog_msg.data
        if self.init_navigator_check:
            self.diagnostic.add("last_nav_data", str(rospy.Time.now().to_sec() - self.last_navigator_callback))
        self.diagnostic.set_level(DiagnosticStatus.OK)

    def update_mission_status(self, mission_status):
        """Update captain status information."""
        self.status.active_controller = mission_status.active_controller
        self.status.altitude_mode = mission_status.altitude_mode
        self.status.mission_active = mission_status.mission_active
        self.status.current_step = mission_status.current_step
        self.status.total_steps = mission_status.total_steps

    def update_captain_status(self, captain_status):
        """Update captain status information."""
        self.status.captain_state = captain_status.state
        self.status.captain_message = captain_status.message

    def update_nav_sts(self, nav):
        """Update navigation information."""
        # Update the last time nav data was received
        self.last_navigator_callback = rospy.Time.now().to_sec()
        if not self.init_navigator_check:
            self.init_navigator_check = True

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
        dt = rospy.Duration(secs=20.0)
        for status in diagnostics.status:
            # Get thrusters status
            if __getDiagnostic__(status, self.diag_thrusters_enabled[0]):
                if __getDiagnostic__(status, self.diag_thrusters_enabled[0], self.diag_thrusters_enabled[1], 'False') == 'True':
                    self.status.thrusters_enabled = True
                else:
                    self.status.thrusters_enabled = False

            # Get vehicle initialized status
            if __getDiagnostic__(status, self.diag_vehicle_initialized[0]):
                if __getDiagnostic__(status, self.diag_vehicle_initialized[0], self.diag_vehicle_initialized[1], 'False') == 'True':
                    self.status.vehicle_initialized = True
                else:
                    self.status.vehicle_initialized = False

            # # Get battery charge
            # if __getDiagnostic__(status, self.diag_battery_charge[0]):
            #     self.status.battery_charge = float(__getDiagnostic__(status, self.diag_battery_charge[0], self.diag_battery_charge[1], 100.0))
            #     self.last_battery_charge = rospy.Time.now()
            # if (rospy.Time.now() - self.last_battery_charge) > dt:
            #     self.status.battery_charge = 0.0

            # Get battery voltage
            # if __getDiagnostic__(status, self.diag_battery_voltage[0]):
            #     self.status.battery_voltage = float(__getDiagnostic__(status, self.diag_battery_voltage[0], self.diag_battery_voltage[1], 100.0))
            #     self.last_battery_voltage = rospy.Time.now()
            # if (rospy.Time.now() - self.last_battery_voltage) > dt:
            #     self.status.battery_charge = 0.0

            # Get IMU data age
            if __getDiagnostic__(status, self.diag_imu_data_age[0]):
                imu_age = float(__getDiagnostic__(status, self.diag_imu_data_age[0], self.diag_imu_data_age[1], 0.0))
                if abs(imu_age) > 10000.0:  # To avoid initialization problems
                    imu_age = 0.0
                self.status.imu_data_age = imu_age

            # Get depth data age from navigator
            if __getDiagnostic__(status, self.diag_depth_data_age[0]):
                depth_age = float(__getDiagnostic__(status, self.diag_depth_data_age[0], self.diag_depth_data_age[1], 0.0))
                if abs(depth_age) > 10000.0:  # To avoid initialization problems
                    depth_age = 0.0
                self.status.depth_data_age = depth_age

            # Get altitude data age from navigator
            if __getDiagnostic__(status, self.diag_altitude_data_age[0]):
                altitude_age = float(__getDiagnostic__(status, self.diag_altitude_data_age[0], self.diag_altitude_data_age[1], 0.0))
                if abs(altitude_age) > 10000.0:  # To avoid initialization problems
                    altitude_age = 0.0
                self.status.altitude_data_age = altitude_age

            # Get DVL data age from navigator
            if __getDiagnostic__(status, self.diag_dvl_data_age[0]):
                dvl_age = float(__getDiagnostic__(status, self.diag_dvl_data_age[0], self.diag_dvl_data_age[1], 0.0))
                if abs(dvl_age) > 10000.0:  # To avoid initialization problems
                    dvl_age = 0.0
                self.status.dvl_data_age = dvl_age

            # Get GPS data age from navigator
            if __getDiagnostic__(status, self.diag_gps_data_age[0]):
                gps_age = float(__getDiagnostic__(status, self.diag_gps_data_age[0], self.diag_gps_data_age[1], 0.0))
                if abs(gps_age) > 10000.0:  # To avoid initialization problems
                    gps_age = 0.0
                self.status.gps_data_age = gps_age

            # Get Navigation data age
            if __getDiagnostic__(status, self.diag_navigation_data_age[0]):
                nav_age = float(__getDiagnostic__(status, self.diag_navigation_data_age[0], self.diag_navigation_data_age[1], 0.0))
                if abs(nav_age) > 10000.0:  # To avoid initialization problems
                    nav_age = 0.0
                self.status.navigation_data_age = nav_age

            # Get DVL valid data age
            if __getDiagnostic__(status, self.diag_dvl_valid_data_age[0]):
                dvl_age = float(__getDiagnostic__(status, self.diag_dvl_valid_data_age[0], self.diag_dvl_valid_data_age[1], 0.0))
                if abs(dvl_age) > 100000.0:
                    dvl_age = 0.0
                self.status.dvl_valid_data_age = dvl_age

            # Get WIFI age
            if __getDiagnostic__(status, self.diag_wifi_data_age[0]):
                wifi_age = float(__getDiagnostic__(status, self.diag_wifi_data_age[0], self.diag_wifi_data_age[1], 0.0))
                if abs(wifi_age) > 100000.0:
                    wifi_age = 0.0
                self.status.wifi_data_age = wifi_age

            # Get modem data age
            if __getDiagnostic__(status, self.diag_modem_data_age[0]):
                modem_age = float(__getDiagnostic__(status, self.diag_modem_data_age[0], self.diag_modem_data_age[1], 0.0))
                if abs(modem_age) > 100000.0:
                    modem_age = 0.0
                self.status.modem_data_age = modem_age

            # Temperatures
            for i in range(0, len(self.diag_temperature)):
                if __getDiagnostic__(status, self.diag_temperature[i][0]):
                    self.status.temperature[i] = float(__getDiagnostic__(status, self.diag_temperature[i][0], self.diag_temperature[i][1], 0.0))
                    self.last_temperature[i] = rospy.Time.now()
	        if (rospy.Time.now() - self.last_temperature[i]) > dt:
	            self.status.temperature[i] = -1000

            # Water inside
            # for i in range(0, len(self.diag_water)):
            #     if __getDiagnostic__(status, self.diag_water[i][0]):
            #         self.water[i] = __getDiagnostic__(status, self.diag_water[i][0], self.diag_water[i][1], 'False') == 'True'
            #         self.last_water_inside = rospy.Time.now()
            # if any(self.water):
            #     self.status.water_detected = True
            # else:
            #     self.status.water_detected = False
            # if (rospy.Time.now() - self.last_water_inside) > dt:
            #     self.status.water_detected = True
            self.status.water_detected = False

            # Get virtual cage status
            if __getDiagnostic__(status, self.diag_inside_virtual_cage[0]):
                if __getDiagnostic__(status, self.diag_inside_virtual_cage[0], self.diag_inside_virtual_cage[1], 'False') == 'True':
                    self.status.inside_virtual_cage = True
                else:
                    self.status.inside_virtual_cage = False

        # Publish status message
        self.status.header.stamp = rospy.Time.now()
        self.status.header.frame_id = rospy.get_namespace() + str('base_link')
        # Fill fixed temperature ids to be published inside the VehicleStatus message
        self.status.temperature_name = self.temperature_name
        self.pub_vehicle_sts.publish(self.status)

    def get_config(self):
        """ Read parameters from ROS Param Server """

        ns = rospy.get_namespace()

        param_dict = {'diag_thrusters_enabled': ('thrusters_enabled', ["",""]),
                      'diag_vehicle_initialized': ('vehicle_initialized', ["",""]),
                      'diag_battery_charge': ('battery_charge', ["",""]),
                      'diag_battery_voltage': ('battery_voltage', ["",""]),
                      'diag_imu_data_age': ('imu_data_age', ["",""]),
                      'diag_depth_data_age': ('depth_data_age', ["",""]),
                      'diag_altitude_data_age': ('altitude_data_age', ["",""]),
                      'diag_dvl_data_age': ('dvl_data_age', ["",""]),
                      'diag_gps_data_age': ('gps_data_age', ["",""]),
                      'diag_navigation_data_age': ('navigation_data_age', ["",""]),
                      'diag_dvl_valid_data_age': ('dvl_valid_data_age', ["",""]),
                      'diag_wifi_data_age': ('wifi_data_age', ["",""]),
                      'diag_modem_data_age': ('modem_data_age', ["",""]),
                      'diag_temperature': ('temperature', [["",""],["",""]]),
                      'diag_water': ('water', [["", ""], ["", ""]]),
                      'temperature_name': ('temperature_name', ["",""]),
                      'diag_inside_virtual_cage': ('inside_virtual_cage', ["",""])
                     }

        param_loader.get_ros_params(self, param_dict)


def __getDiagnostic__(status, name, key='none', default=0.0):
    if status.level == DiagnosticStatus.STALE:
        return False
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
