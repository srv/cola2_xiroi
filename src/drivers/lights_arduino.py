#!/usr/bin/env python

"""
    A ROS Node for the Arduino microcontroller

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from ros_arduino_python.arduino_driver import Arduino
import os, time
import thread
from serial.serialutil import SerialException
from xiroi.msg import BatteryInfo

class ArduinoROS():
    def __init__(self):
        rospy.init_node('arduino_node', log_level=rospy.INFO)

        # Get the actual node name in case it is set in the launch file
        self.name = rospy.get_name()

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        self.port = rospy.get_param("~port", "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A6008iB3-if00-port0")
        # self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # Callback for BatteryInfo
        rospy.Subscriber("/state_of_charge", BatteryInfo, self.blinking_leds)
        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout)
        # Make the connection
        self.controller.connect()

        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")

        while not rospy.is_shutdown():
            r.sleep()

    # Service callback functions
    def blinking_leds(self, data):
        self.controller.digital_write(2, 0)

        self.full_charge = 12.65        # 100%
        self.mediumUP_charge = 12.45    # 75%
        self.mediumDOWN_charge = 12.24  # 50%
        self.low_charge = 12.06         # 25%

        if (data.state_of_charge==100):
            for i in range(4):
                self.controller.digital_write(2, 1)
                time.sleep(1.0)
                self.controller.digital_write(2, 0)

        if (data.state_of_charge==75):
            for i in range(3):
                self.controller.digital_write(2, 1)
                time.sleep(1.0)
                self.controller.digital_write(2, 0)

        if (data.state_of_charge==25):
            for i in range(2):
                self.controller.digital_write(2, 1)
                time.sleep(1.0)
                self.controller.digital_write(2, 0)

    def shutdown(self):
        rospy.loginfo("Shutting down Arduino Node...")

        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            rospy.sleep(2)
        except:
            pass

        # Close the serial port
        try:
            self.controller.close()
        except:
            pass
        finally:
            rospy.loginfo("Serial port closed.")
            os._exit(0)

if __name__ == '__main__':


    try:
        ArduinoROS()
    except SerialException:
        rospy.logerr("Serial exception trying to open port.")
        os._exit(0)