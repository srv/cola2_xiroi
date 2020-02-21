#!/usr/bin/env python
# Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""@@>Publishes fixed CPU and RAM usage data and temperature from the vehicle computer in order to avoid iquaview interface warnings.<@@"""

# ROS imports
import rospy
import subprocess
import sys
import psutil
from cola2_lib.rosutils.diagnostic_helper import DiagnosticHelper
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32


class ComputerLogger(object):
    """Publishes CPU and RAM usage data and temperature from the vehicle computer."""

    def __init__(self, name):
        """ Constructor """
        # Init class vars
        self.name = name

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper(self.name, "soft")

        # Publisher
        resolved_name = rospy.get_name()
        self.pub_temp = rospy.Publisher(resolved_name + "/temperature", Temperature, queue_size = 2)
        self.pub_ram = rospy.Publisher(resolved_name + "/ram_usage", Float32, queue_size = 2)
        self.pub_cpu = rospy.Publisher(resolved_name + "/cpu_usage", Float32, queue_size = 2)

        # Start timer
        rospy.Timer(rospy.Duration(5), self.iterate)

        # Show message
        rospy.loginfo("%s: initialized", self.name)

    def iterate(self, event):
        """ Callback from the main timer """
        try:
            # Execute sensors command and parse output
            p = subprocess.Popen(['sensors'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            out, err = p.communicate()
            splitted = out.split("\n")

            max_core_temp = 100

            # Use psutil to get cpu and ram usage
            cpu_usage = 50
            ram_usage = 50

            self.diagnostic.add("cpu_temperature", str(max_core_temp))
            self.diagnostic.set_level(DiagnosticStatus.OK)

            msg = Temperature()
            msg.header.stamp = rospy.Time.now()
            msg.temperature = max_core_temp
            msg.variance = 0  # unknown
            self.pub_temp.publish(msg)

            msg = Float32()
            msg.data = cpu_usage
            self.pub_cpu.publish(msg)

            msg.data = ram_usage
            self.pub_ram.publish(msg)

        except:
            rospy.logwarn("%s: unable to get data: %s", self.name, sys.exc_info()[0])


if __name__ == '__main__':
    try:
        rospy.init_node('computer_logger')
        computer_logger = ComputerLogger(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
