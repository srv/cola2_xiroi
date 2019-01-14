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
        self.base_frame = rospy.get_param("~base_frame", 'base_link')
        self.left_thruster_pin = rospy.get_param("~left_thruster_pin", 0)
        self.right_thruster_pin = rospy.get_param("~right_thruster_pin", 1)
        self.current_sensor_pin = rospy.get_param("~current_sensor_pin", 0)
        self.current_sensor_power_pin = rospy.get_param("~current_sensor_power_pin", 3)
        self.first_time=True;

        #Slew rate parameters
        self.rising_rate = 0.3 #0.3
        self.falling_rate = 1.0 #1
        self.setpoint_limit = 0.6 #1

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # Callback for thrusters
        rospy.Subscriber("/cola2_control/thrusters_data", Setpoints, self.thruster_callback)
        self.setpoints_corrected = rospy.Publisher("setpoints_corrected", Setpoints, queue_size=1)
        self.thruster_current = rospy.Publisher("thruster_current",Current, queue_size=1)

        #Services
        self.thrusters_enabled = False
        # Create services
        self.e_srv = rospy.Service('control/enable_thrusters', Empty, self.enable_thrusters_srv)
        self.d_srv = rospy.Service('control/disable_thrusters', Empty, self.disable_thrusters_srv)
     
        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout)
        # Make the connection
        self.controller.connect()
        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")

        self.controller.pin_mode(self.current_sensor_power_pin, 1)
        self.controller.digital_write(self.current_sensor_power_pin,1)

        while not rospy.is_shutdown():
            current = self.controller.analog_read(self.current_sensor_pin)
            # publish
            msg = Current()
            msg.header.stamp = rospy.Time.now()
            msg.current = ((current/1024.0*5.0) - 2.41)/0.066
            self.thruster_current.publish(msg)
            r.sleep()

    def disable_thrusters_srv(self,req):
        """ Disable thruster service """
        self.controller.servo_write(self.left_thruster_pin,1500)
        self.controller.servo_write(self.right_thruster_pin,1500)
        self.thrusters_enabled = False
        rospy.loginfo("THRUSTERS DISABLED")
        return EmptyResponse()
    

    def enable_thrusters_srv(self,req):
        """ Enable thruster service """
        self.thrusters_enabled = True
        rospy.loginfo("THRUSTERS ENABLED")
        return EmptyResponse()


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
        msg0=msg.setpoints[0]*0;
        msg1=msg.setpoints[1];
        msg2=msg.setpoints[2];
        
        # time=msg.header.stamp.to_sec()
        time=rospy.Time.now().to_sec()

        if(self.first_time):
            self.controller.servo_write(self.left_thruster_pin,1500+msg0*400)
            self.controller.servo_write(self.right_thruster_pin,1500+msg1*400)
            self.old_time = time
            self.old_msg0 = msg0
            self.old_msg1 = msg1
            self.first_time = False
            return

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

        if self.thrusters_enabled:
            print str(1500+msg0*400) + ' ' + str(1500+msg1*400)
            self.controller.servo_write(self.left_thruster_pin,1500+msg0*400)
            self.controller.servo_write(self.right_thruster_pin,1500+msg1*400)

        self.old_time = time
        self.old_msg0 = msg0
        self.old_msg1 = msg1

        msg.setpoints = [msg0, msg1]
        self.setpoints_corrected.publish(msg)

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
