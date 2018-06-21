#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


"""@@This node is used to manage Sparus II board controlling the flash and the antenna leds.@@"""


# ROS imports
import roslib
import rospy
import serial
from std_srvs.srv import Empty, EmptyResponse
from cola2_lib.rosutils import param_loader
from cola2_lib.rosutils.diagnostic_helper import DiagnosticHelper
from diagnostic_msgs.msg import DiagnosticStatus


# MONCONTROL BOARD COMMANDS
# V   --> version number
# W   --> 0 or 1
# B   --> battery level
# L0  --> disable green led
# L1  --> enable green led
# F0  --> disable flash
# F1  --> enable flash
# FD0 --> disable flash by default
# FD1 --> enable flash by default


class MonControlBoard(object):
    """ This node is used to manage Sparus II board """

    def __init__(self, name):
        """ Init the class """
        self.name = name

        # Get config from param server
        self.get_config()

        # Open serial port
        self.open_serial_port()

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper(self.name, "hw")

        # Create timer to check water inside
        rospy.Timer(rospy.Duration(2.0), self.check_water)

        # Define services
        self.enable_green_led_srv = rospy.Service(
                                    '/cola2_safety/enable_green_led',
                                    Empty,
                                    self.enable_green_led)
        self.disable_green_led_srv = rospy.Service(
                                    '/cola2_safety/disable_green_led',
                                    Empty,
                                    self.disable_green_led)
        self.enable_flash_srv = rospy.Service(
                                    '/cola2_safety/enable_flash',
                                    Empty,
                                    self.enable_flash)
        self.disable_flash_srv = rospy.Service(
                                    '/cola2_safety/disable_flash',
                                    Empty,
                                    self.disable_flash)
        self.enable_flash_by_default_srv = rospy.Service(
                                    '/cola2_safety/enable_flash_by_default',
                                    Empty,
                                    self.enable_flash_by_default)
        self.disable_flash_by_default_srv = rospy.Service(
                                    '/cola2_safety/disable_flash_by_default',
                                    Empty,
                                    self.disable_flash_by_default)
        self.mon_control_board_version_srv = rospy.Service(
                                    '/cola2_safety/mon_control_board_version',
                                    Empty,
                                    self.mon_control_board_version)
        self.mon_control_board_battery_level_srv = rospy.Service(
                                    '/cola2_safety/mon_control_board_battery_level',
                                    Empty,
                                    self.mon_control_board_battery_level)

        # Show message
        rospy.loginfo('%s: initialized', self.name)


    def open_serial_port(self):
        """ Method used to open serial port """
        # Open serial port with timeout of 1 seconds
        try:
            rospy.loginfo("%s: opening serial port", self.name)
            self.serial_port = serial.Serial(self.serial_path,
                                             self.serial_baud_rate,
                                             timeout = 1)
            rospy.loginfo("%s: serial port connected", self.name)
        except serial.serialutil.SerialException:
            rospy.logerr("%s: problem openning serial port: %s at %s",
                         self.name, self.serial_path, self.serial_baud_rate)

        return self.serial_port.isOpen()


    def check_water(self, event):
        """ Check if there is water inside """
        if self.serial_port.isOpen():
            # Write and read response
            try:
                self.serial_port.write('W\n\r')
            except:
                rospy.logerr('%s: error sending request', self.name)
            try:
                water = self.serial_port.readline()
            except:
                rospy.logerr('%s: error reading line', self.name)
                water = 'exception'

            # Check response
            if 'W=0' in water:
                self.diagnostic.add("water_inside", "False")
                self.diagnostic.set_level(DiagnosticStatus.OK ,'No water')
            elif 'W=1' in water:
                for x in range(20):
                    rospy.logfatal('%s: WATER INSIDE! WATER INSIDE! WATER INSIDE! ', self.name)
                self.diagnostic.add("water_inside", "True")
                self.diagnostic.set_level(DiagnosticStatus.ERROR ,'Water detected')
            else:
                self.diagnostic.add("water_inside", "Unknown")
                self.diagnostic.set_level(DiagnosticStatus.WARN ,'Unable to parse')
        else:
            rospy.logerr('%s: mon_control_board serial port not opened', self.name)
            self.open_serial_port()


    def enable_green_led(self, req):
        """ Enable green led """
        rospy.loginfo('%s: enabling green led', self.name)
        if self.serial_port.isOpen():
            try:
                self.serial_port.write('L1\n\r')
            except:
                rospy.logerr('%s: error sending request', self.name)
        else:
            rospy.logerr('%s: mon_control_board serial port not opened', self.name)
        return EmptyResponse()


    def disable_green_led(self, req):
        """ Disable green led """
        rospy.loginfo('%s: disabling green led', self.name)
        if self.serial_port.isOpen():
            try:
                self.serial_port.write('L0\n\r')
            except:
                rospy.logerr('%s: error sending request', self.name)
        else:
            rospy.logerr('%s: mon_control_board serial port not opened', self.name)
        return EmptyResponse()


    def enable_flash(self, req):
        """ Enable flash """
        rospy.loginfo('%s: enabling flash', self.name)
        if self.serial_port.isOpen():
            try:
                self.serial_port.write('F1\n\r')
            except:
                rospy.logerr('%s: error sending request', self.name)
        else:
            rospy.logerr('%s: mon_control_board serial port not opened', self.name)
        return EmptyResponse()


    def disable_flash(self, req):
        """ Disable flash """
        rospy.loginfo('%s: disabling flash', self.name)
        if self.serial_port.isOpen():
            try:
                self.serial_port.write('F0\n\r')
            except:
                rospy.logerr('%s: error sending request', self.name)
        else:
            rospy.logerr('%s: mon_control_board serial port not opened', self.name)
        return EmptyResponse()


    def enable_flash_by_default(self, req):
        """ Enable flash by default """
        rospy.loginfo('%s: enabling flash by default', self.name)
        if self.serial_port.isOpen():
            try:
                self.serial_port.write('FD1\n\r')
            except:
                rospy.logerr('%s: error sending request', self.name)
        else:
            rospy.logerr('%s: mon_control_board serial port not opened', self.name)
        return EmptyResponse()


    def disable_flash_by_default(self, req):
        """ Disable flash by default """
        rospy.loginfo('%s: disabling flash by default', self.name)
        if self.serial_port.isOpen():
            try:
                self.serial_port.write('FD0\n\r')
            except:
                rospy.logerr('%s: error sending request', self.name)
        else:
            rospy.logerr('%s: mon_control_board serial port not opened', self.name)
        return EmptyResponse()


    def mon_control_board_version(self, req):
        """ mon_control_board version """
        rospy.loginfo('%s: requesting mon_control_board version', self.name)
        if self.serial_port.isOpen():
            try:
                self.serial_port.write('V\n\r')
            except:
                rospy.logerr('%s: error sending request', self.name)

            try:
                rospy.loginfo('%s: mon_control_board version is: %s', self.name, self.serial_port.readline())
            except:
                rospy.logerr('%s: error reading line', self.name)
        else:
            rospy.logerr('%s: mon_control_board serial port not opened', self.name)
        return EmptyResponse()


    def mon_control_board_battery_level(self, req):
        """ mon_control_board version """
        rospy.loginfo('%s: requesting mon_control_board battery level', self.name)
        if self.serial_port.isOpen():
            try:
                self.serial_port.write('B\n\r')
            except:
                rospy.logerr('%s: error sending request', self.name)

            try:
                rospy.loginfo('%s: mon_control_board battery level is: %s', self.name, self.serial_port.readline())
            except:
                rospy.logerr('%s: error reading line', self.name)
        else:
            rospy.logerr('%s: mon_control_board serial port not opened', self.name)
        return EmptyResponse()


    def get_config(self):
        """ Get config from param server """
        param_dict = {'serial_path': ('serial_port/path', "/dev/ttyS9"),
                      'serial_baud_rate': ('serial_port/baud_rate', 19200)}

        if not param_loader.get_ros_params(self, param_dict):
            self.bad_config_timer = rospy.Timer(rospy.Duration(0.4), self.bad_config_message)


    def bad_config_message(self, event):
        """ Timer to show an error if loading parameters failed """
        rospy.logfatal('%s: bad parameters in param server!', self.name)


if __name__ == '__main__':
    try:
        rospy.init_node('s2_mon_control_board')
        mon_control_board = MonControlBoard(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
