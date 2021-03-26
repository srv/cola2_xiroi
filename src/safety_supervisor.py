#!/usr/bin/env python
# Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""@@> The diagnostics supervisor receives the vehicle status message and applies a set of rules aimed at detecting errors in the system (low battery level, detection from water leak sensors, etc.). These rules are based on the configuration values set on the safety.yaml file of the vehicles. If any of the rule checks is triggered, the diagnostics supervisor calls a recovery action, which acts appropriately depending on the type of error.<@@"""

import rospy
import dynamic_reconfigure.client
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from diagnostic_msgs.msg import DiagnosticStatus
from cola2_msgs.msg import RecoveryAction, VehicleStatus, SafetySupervisorStatus
from cola2_msgs.srv import Recovery, RecoveryRequest
from status_code import StatusCode
from cola2_lib.rosutils.diagnostic_helper import DiagnosticHelper
from cola2_lib.rosutils import param_loader

# Time during which to show the message for external recovery
TIME_SHOW_EXTERNAL_RECOVERY = 20


class SafetySupervisor(object):
    """
    The diagnostics supervisor receives the vehicle status message and applies a set of rules aimed at detecting
    errors in the system (low battery level, detection from water leak sensors, etc.).
    These rules are based on the configuration values set on the safety.yaml file of the vehicles.
    If any of the rule checks is triggered, the diagnostics supervisor calls a recovery action, which acts
    appropriately depending on the type of error.
    It also handles recovery actions called externally (using the recovery_actions/recover service
    through the CLI or the GUI).
    """
    def __init__(self, name):
        """ Init the class. """
        self.name = name

        # Flag to know if vehicle is initialized
        self.vehicle_init = False

        # Init status code (4 bytes) to 0s
        self.status_code = ['0' for i in range(32)]

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper(self.name, "soft")

        # Flags to know if a recovery action is ongoing
        self.is_recovery_enabled = False
        self.is_external_recovery_enabled = False

        # Initialize recovery action message
        self.ra_msg = RecoveryAction()
        self.ra_msg.header.stamp = rospy.Time.now()
        self.ra_msg.error_level = RecoveryAction.NONE
        self.ra_msg.error_string = ""
        self.old_level = RecoveryAction.NONE
        self.old_str_err = ""

        self.timeout_reset = 10

        # Get config parameters
        self.get_config()

        # Create Publisher
        self.pub_safety_supervisor_state = rospy.Publisher(rospy.get_name() + '/status',
                                                           SafetySupervisorStatus, queue_size=2)

        # Service client for recovery actions
        ns = rospy.get_namespace()
        try:
            rospy.wait_for_service(ns + 'recovery_actions/recover', 20)
            self.recover_action_srv = rospy.ServiceProxy(ns + 'recovery_actions/recover', Recovery)
        except rospy.exceptions.ROSException:
            rospy.logerr('Error creating client to recovery action.')
            rospy.signal_shutdown('Error creating recover action client')

        # Service client to publish parameters
        publish_params_srv_name = ns + 'param_logger/publish_params'
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(publish_params_srv_name, 5)
                self.publish_params_srv = rospy.ServiceProxy(publish_params_srv_name, Trigger)
                break
            except rospy.exceptions.ROSException:
                rospy.loginfo('Waiting for client to service %s', publish_params_srv_name)

        # Subscriber
        rospy.Subscriber(ns + "vehicle_status",
                         VehicleStatus,
                         self.check_vehicle_status,
                         queue_size=1)

        # To handle recovery actions that have not been called from safety_supervisor
        rospy.Subscriber(ns + "recovery_actions/external_recovery_action",
                         RecoveryAction,
                         self.external_recovery_action,
                         queue_size=1)

        # Create service to reload safety parameters
        self.reload_params_srv = rospy.Service(rospy.get_name() + '/reload_params',
                                               Empty,
                                               self.reload_params_srv)
        rospy.loginfo('Initialized')

    def reload_params_srv(self, req):
        """ Callback of reload params service """
        rospy.loginfo('Received reload params service')
        self.get_config()

        req = TriggerRequest()
        res = self.publish_params_srv(req)
        if not res.success:
            rospy.logwarn('Publish params did not succeed -> %s', res.message)
        return EmptyResponse()

    def update_current_step_byte(self, current_step):
        """ Update status code with the current step information. """
        current_step = bin(current_step % 256)
        # Delete previous bits
        for b in range(8):
            self.status_code[StatusCode.CURRENT_MISSION_STEPS_BASE - b] = '0'
        # Fill current step
        for b in range(len(current_step) - 2):
            self.status_code[StatusCode.CURRENT_MISSION_STEPS_BASE - b] = current_step[-(1 + b)]

    def external_recovery_action(self, recovery_action):
        """ Callback for external recovery action subscriber """
        rospy.loginfo("External recovery action received")
        self.ra_msg = recovery_action
        self.old_level = recovery_action.error_level
        self.old_str_err = recovery_action.error_string
        self.is_external_recovery_enabled = True

    def check_vehicle_status(self, vehicle_status):
        """
        Check vehicle status fields to see if any of the following rules is true.
        When a rule is triggered call the appropriated recovery action.
        """

        self.status_code = ['0' for i in range(32)]

        # Flag to keep track if any recovery action is triggered
        self.is_recovery_enabled = False

        # uint16 NONE = 0
        # uint16 INFORMATIVE = 1
        # uint16 ABORT_MISSION = 2
        # uint16 ABORT_AND_SURFACE = 3
        # uint16 EMERGENCY_SURFACE = 4
        # uint16 STOP_THRUSTERS = 5

        # Rule: Init vehicle
        if vehicle_status.vehicle_initialized:
            self.vehicle_init = True

        # Rule: Battery Level
        battery_charge = vehicle_status.battery_charge
        battery_voltage = vehicle_status.battery_voltage
        self.diagnostic.add('battery_charge', str(battery_charge))
        if battery_charge < self.min_battery_charge or battery_voltage < self.min_battery_voltage:
            self.status_code[StatusCode.BATTERY_ERROR] = '1'
            self.call_recovery_action("Battery Level below threshold!", RecoveryAction.INFORMATIVE)
        elif battery_charge < 1.5*self.min_battery_charge:
            self.status_code[StatusCode.LOW_BATTERY_WARNING] = '1'
            self.call_recovery_action("Battery Level Low", RecoveryAction.INFORMATIVE)

        # Rule: No IMU data
        last_imu = vehicle_status.imu_data_age
        self.diagnostic.add('last_imu_data', str(last_imu))
        if last_imu > self.min_imu_update:
            rospy.logerr("No IMU data since %s", str(last_imu))
            self.status_code[StatusCode.NAVIGATION_ERROR] = '1'
            self.call_recovery_action("No IMU data!", RecoveryAction.INFORMATIVE)

       # Rule: No GPS data
        last_gps = vehicle_status.gps_data_age
        self.diagnostic.add('last_gps_data', str(last_gps))
        if (last_gps > self.min_gps_update) and vehicle_status.at_surface:
            self.call_recovery_action("No GPS data!", RecoveryAction.INFORMATIVE)

        # Rule: No Navigation data
        last_nav = vehicle_status.navigation_data_age
        self.diagnostic.add('last_nav_data', str(last_nav))
        if last_nav > self.min_nav_update:
            self.status_code[StatusCode.NAVIGATION_ERROR] = '1'
            self.call_recovery_action("No Navigation data!", RecoveryAction.INFORMATIVE)

        # Rule: No WIFI data and not in a mission
        last_ack = vehicle_status.wifi_data_age
        if not vehicle_status.mission_active:
            self.diagnostic.add('last_ack', str(last_ack))
            if last_ack > self.min_wifi_update:
                self.call_recovery_action("No WiFi data!", RecoveryAction.INFORMATIVE)
        else:
            #rospy.loginfo("A mission is active. WiFi timeout disabled.")
            self.diagnostic.add('last_ack', 'Mission active')

        # Rule: Outside Virtual Cage
        if not vehicle_status.inside_virtual_cage:
            if self.virtual_cage_calls_keep_position:
                self.call_recovery_action("Outside virtual cage!", RecoveryAction.INFORMATIVE)

        # Rule: High Temperature
        temperatures = vehicle_status.temperature
        if temperatures:
            for t in range(0, len(temperatures)):
                if temperatures[t] > self.max_temperatures_values[t]:
                    self.status_code[StatusCode.HIGH_TEMPERATURE] = '1'
                    self.call_recovery_action(vehicle_status.temperature_name[t] + " high temperature",
                                              RecoveryAction.INFORMATIVE)
                else:
                    self.diagnostic.add('vehicle_temperature', 'Ok')

        # Rule: Watchdog
        elapsed_time = vehicle_status.elapsed_time
        self.diagnostic.add('elapsed_time', str(elapsed_time))
        if float(elapsed_time) > self.timeout and self.timeout_reset < 0:
            self.status_code[StatusCode.WATCHDOG_TIMER] = '1'
            self.call_recovery_action("Watchdog timeout reached!", RecoveryAction.INFORMATIVE)
        else:
            self.timeout_reset = self.timeout_reset - 1

        # Update status bytes with the current step information
        self.update_current_step_byte(vehicle_status.current_step)

        # If there is an enabled external recovery action (not triggered by the safety supervisor), show it
        if self.is_external_recovery_enabled:
            # Flag corresponding bit of status code according to recovery action level
            if self.ra_msg.error_level == RecoveryAction.ABORT_MISSION:
                self.status_code[StatusCode.RA_ABORT_MISSION] = '1'
            elif self.ra_msg.error_level == RecoveryAction.INFORMATIVE:
                self.status_code[StatusCode.RA_ABORT_SURFACE] = '1'
            elif self.ra_msg.error_level == RecoveryAction.INFORMATIVE:
                self.status_code[StatusCode.INFORMATIVE] = '1'
            self.is_recovery_enabled = True
            # init timer to show the message for some seconds
            rospy.Timer(rospy.Duration(TIME_SHOW_EXTERNAL_RECOVERY), self.timer_callback, oneshot=True)

        if not self.is_recovery_enabled:
            self.diagnostic.set_level(DiagnosticStatus.OK)
            self.ra_msg.header.stamp = rospy.Time.now()
            self.old_level = self.ra_msg.error_level
            self.old_str_err = self.ra_msg.error_string
            self.ra_msg.error_level = RecoveryAction.NONE
            self.ra_msg.error_string = ""

        # Publish Safety Supervisor Status
        sss_msg = SafetySupervisorStatus()
        sss_msg.header.stamp = rospy.Time.now()
        status_code_str = ''.join(self.status_code)
        status_code_int32 = int(status_code_str, 2)
        sss_msg.status_code = status_code_int32
        sss_msg.recovery_action = self.ra_msg
        self.pub_safety_supervisor_state.publish(sss_msg)

        #print("Status code:------------------------------------------------")
        #sc = StatusCode()
        #print( sc.unpack(status_code_int32) )

    def timer_callback(self, event):
        """ Resets state so that external recovery is not shown anymore """
        self.is_external_recovery_enabled = False

    def get_config(self):
        """ Read parameters from ROS Param Server """

        ns = rospy.get_namespace()

        param_dict = {'min_altitude': (ns + '/safe_depth_altitude/min_altitude', 1.5),
                      'max_depth': (ns + '/safe_depth_altitude/max_depth', 20.0),
                      'min_battery_charge': (ns + '/safety/min_battery_charge', 20.0),
                      'min_battery_voltage': (ns + '/safety/min_battery_voltage', 11.0),
                      'min_imu_update': (ns + '/safety/min_imu_update', 2.0),
                      'min_depth_update': (ns + '/safety/min_depth_update', 2.0),
                      'min_altitude_update': (ns + '/safety/min_altitude_update', 2.0),
                      'min_gps_update': (ns + '/safety/min_gps_update', 2.0),
                      'min_dvl_update': (ns + '/safety/min_dvl_update', 2.0),
                      'min_nav_update': (ns + '/safety/min_nav_update', 2.0),
                      'min_wifi_update': (ns + '/safety/min_wifi_update', 2.0),
                      'working_area_north_origin': (ns + '/virtual_cage/north_origin', -50.0),
                      'working_area_east_origin': (ns + '/virtual_cage/east_origin', -50.0),
                      'working_area_north_length': (ns + '/virtual_cage/north_longitude', 100.0),
                      'working_area_east_length': (ns + '/virtual_cage/east_longitude', 100.0),
                      'virtual_cage_calls_keep_position': (ns + '/virtual_cage/calls_keep_position', False),
                      'timeout': (ns + '/safety/timeout', 3600),
                      'min_modem_update': (ns + '/safety/min_modem_update', 2.0),
                      'min_dvl_good_data': (ns + '/safety/min_dvl_good_data', 30),
                      'max_temperatures_values': (ns + '/safety/max_temperatures_values', [55.0, 80.0, 95.0])}

        param_loader.get_ros_params(self, param_dict)


        # Dynamic reconfigure for defining min altitude and max depth
        try:
            client2 = dynamic_reconfigure.client.Client("safe_depth_altitude", timeout=10)
            client2.update_configuration({"min_altitude": self.min_altitude,
                                          "max_depth": self.max_depth})

        except rospy.exceptions.ROSException:
            rospy.logerr('Error modifying safe_depth_altitude params.')

        # Dynamic reconfigure for defining virtual cage limits
        try:
            client3 = dynamic_reconfigure.client.Client("virtual_cage", timeout=10)
            client3.update_configuration({"north_origin": self.working_area_north_origin,
                                          "east_origin": self.working_area_east_origin,
                                          "north_longitude": self.working_area_north_length,
                                          "east_longitude": self.working_area_east_length,
                                          "enable": True})
        except rospy.exceptions.ROSException:
            rospy.logerr('Error modifying virtual_cage params.')

    def call_recovery_action(self, str_err="Error!", level=RecoveryAction.INFORMATIVE):
        """
        If the vehicle is initialized, calls the recovery action service with the corresponding level and message.
        """
        self.is_recovery_enabled = True
        self.diagnostic.set_level(DiagnosticStatus.ERROR, str_err)

        if self.vehicle_init:
            # If the same recovery action has been called again do not change the recovery_action timestamp
            if level != self.old_level or str_err != self.old_str_err:
                self.ra_msg.header.stamp = rospy.Time.now()
                self.ra_msg.error_level = level
                self.ra_msg.error_string = str_err
            rospy.logerr("%s", str_err)

            # Flag corresponding bit of status code according to recovery action level
            if level == RecoveryAction.ABORT_MISSION:
                self.status_code[StatusCode.RA_ABORT_MISSION] = '1'
            elif level == RecoveryAction.ABORT_AND_SURFACE:
                self.status_code[StatusCode.RA_ABORT_SURFACE] = '1'
            elif level == RecoveryAction.EMERGENCY_SURFACE:
                self.status_code[StatusCode.RA_EMERGENCY_SURFACE] = '1'

            # Call recovery action service
            req = RecoveryRequest()
            ra = RecoveryAction()
            ra.error_level = level
            req.requested_action = ra
            ans = self.recover_action_srv(req)
            rospy.logerr("Recovery action called --> %s", ans)

            # Save last called action
            self.old_level = level
            self.old_str_err = str_err

        rospy.sleep(5.0)


if __name__ == '__main__':
    try:
        rospy.init_node('safety_supervisor')
        SS = SafetySupervisor(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
