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
from cola2_xiroi.msg import Current
from cola2_msgs.msg import Setpoints
from std_srvs.srv import Empty, EmptyResponse

class ArduinoROS():
    def __init__(self):
        rospy.init_node('arduino_node', log_level=rospy.INFO)

        # Get the actual node name in case it is set in the launch file
        self.name = rospy.get_name()

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        self.port = rospy.get_param("~port", "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A6008iB3-if00-port0")
        self.baud = int(rospy.get_param("~baud",57600))
        self.timeout = rospy.get_param("~timeout",0.5)
        self.base_frame = rospy.get_param("base_frame", 'base_link')
        self.left_thruster_pin = rospy.get_param("left_thruster_pin", 0)
        self.right_thruster_pin = rospy.get_param("right_thruster_pin", 1)
        self.current_sensor_pin = rospy.get_param("current_sensor_pin", 0)
        self.current_sensor_power_pin = rospy.get_param("current_sensor_power_pin", 3)
        self.activate_ramp = rospy.get_param("activate_ramp", True)
        self.rising_rate = rospy.get_param("rising_rate", 0.5)
        self.falling_rate = rospy.get_param("falling_rate", 1.0)
        self.setpoint_limit = rospy.get_param("setpoint_limit", 0.9)
        self.zero_pwm = rospy.get_param("zero_pwm", 1500)
        self.max_pwm = rospy.get_param("max_pwm", 1700)
        self.first_time=True

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # Callback for thrusters
        rospy.Subscriber("setpoints", Setpoints, self.thruster_callback)

        # Publishers
        self.setpoints_corrected = rospy.Publisher("controller/setpoints_corrected", Setpoints, queue_size=1)
        self.thruster_current = rospy.Publisher("controller/thruster_current",Current, queue_size=1)
     
        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout)
        # Make the connection
        self.controller.connect()
        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")

        self.controller.digital_pin_mode(self.current_sensor_power_pin, 1)
        self.controller.digital_write(self.current_sensor_power_pin,1)

        # TODO: Timer
        while not rospy.is_shutdown():
            current = self.controller.analog_read(self.current_sensor_pin)
            # publish
            msg = Current()
            msg.header.stamp = rospy.Time.now()
            msg.current = ((current/1024.0*5.0) - 2.41)/0.066
            self.thruster_current.publish(msg)
            r.sleep()


    def limitSetpoint(self, msg, time, old_msg, old_time):
        #Rate calculation
        rate = (msg - old_msg)/(time - old_time)

        # Case 1
        if (rate>0 and msg>0 and rate > self.rising_rate):
            msg = (time-old_time)*self.rising_rate + old_msg
        # Case 2
        elif (rate<0 and msg>=0 and rate < -self.falling_rate):
            msg = -(time-old_time)*self.falling_rate + old_msg
        # Case 3
        elif (rate<0 and msg<0  and rate < -self.rising_rate):
            msg = -(time-old_time)*self.rising_rate + old_msg
        # Case 4
        elif (rate>0 and msg<=0 and rate > self.falling_rate):
            msg = (time-old_time)*self.falling_rate + old_msg

        return msg

    # Service callback functions
    def thruster_callback(self, msg):
    #     #1900 => max thrust positive
    #     #1500 => 0A
    #     #1100 => max thrust negative
        actual_time=rospy.Time.now()
        message_time=msg.header.stamp
        delay_threshold=rospy.Duration(0,650000)
        time_dif=actual_time-message_time

        # if time_dif<delay_threshold:

        #extract the second and thirth data from the setpoints message 
        msg0=msg.setpoints[0]
        msg1=msg.setpoints[1]

        
        # time=msg.header.stamp.to_sec()
        time=rospy.Time.now().to_sec()

        if(self.first_time):
            self.controller.servo_write(self.left_thruster_pin,self.setpoint_conversion(msg0))
            self.controller.servo_write(self.right_thruster_pin,self.setpoint_conversion(msg1))
            self.old_time = time
            self.old_msg0 = msg0
            self.old_msg1 = msg1
            self.first_time = False
            return


        if self.activate_ramp:
            msg0 = self.limitSetpoint(msg0, time, self.old_msg0, self.old_time)
            msg1 = self.limitSetpoint(msg1, time, self.old_msg1, self.old_time)

            if msg0 > self.setpoint_limit:
                msg0 = self.setpoint_limit
            elif msg0 < -self.setpoint_limit:
                msg0 = -self.setpoint_limit

            if msg1 > self.setpoint_limit:
                msg1 = self.setpoint_limit
            elif msg1 < -self.setpoint_limit:
                msg1 = -self.setpoint_limit

            self.old_time = time
            self.old_msg0 = msg0
            self.old_msg1 = msg1


        # Setpoint to thrusters
        # print self.setpoint_conversion(msg0), self.setpoint_conversion(msg1)
        self.controller.servo_write(self.left_thruster_pin, self.setpoint_conversion(msg0))
        self.controller.servo_write(self.right_thruster_pin, self.setpoint_conversion(msg1))
        corrected = Setpoints()
        corrected.header.stamp = msg.header.stamp
        corrected.header.seq = msg.header.seq
        corrected.setpoints = [msg0, msg1]
        self.setpoints_corrected.publish(corrected)

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

    def setpoint_conversion(self, setpoint):
        return self.zero_pwm + setpoint*(self.max_pwm-self.zero_pwm)
        

if __name__ == '__main__':

    try:
        ArduinoROS()
    except SerialException:
        rospy.logerr("Serial exception trying to open port.")
        os._exit(0)
