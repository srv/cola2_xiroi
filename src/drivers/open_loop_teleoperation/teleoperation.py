#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('cola2_xiroi')
from cola2_msgs.msg import Setpoints
from sensor_msgs.msg import Joy
from math import *
import numpy as np
from std_msgs.msg import String

class Controller:
    def __init__(self):
        self.seq = 0
        self.map_ack_init = False
        self.map_ack_alive = True
        self.last_map_ack = 0.0

        # Publisher
        self.pub_thrusters_setpoints = rospy.Publisher(
            '/xiroi/setpoints_req',
            Setpoints,
            queue_size = 1)

        self.pub_check_joystick = rospy.Publisher(
            '/xiroi/control/ack',
            String,
            queue_size=2)

        # Subscriber
        rospy.Subscriber("/xiroi/control/ack_joy",
                         Joy,
                         self.joystick_data_callback,
                         queue_size=1)

        rospy.Subscriber("/xiroi/control/ack_ack",
                         String,
                         self.ack_ack_callback,
                         queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.check_map_ack)

        # Message
        self.msg = Setpoints()
        self.msg.header.frame_id = 'xiroi'
        # self.msg.requester = 'teleoperation'
        # self.msg.priority = 70
        self.msg.setpoints = np.array([0.0, 0.0])

    def ack_ack_callback(self, ack_msg):
        """ This is the callback for the ack safety message """
        data = ack_msg.data.split(' ')
        if data[1] == 'ack' and data[0] == str(self.seq + 1):
            self.map_ack_alive = True
            self.map_ack_init = True
            self.seq = self.seq + 1
            self.last_map_ack = rospy.Time.now().to_sec()


    def joystick_data_callback(self, data):
        # rospy.loginfo( data.axes)
        forward = data.axes[6]
        clockwise_yaw = data.axes[11]
        aux = np.array([0.0, 0.0])
        aux[0] = forward + clockwise_yaw
        aux[1] = forward - clockwise_yaw
        self.msg.setpoints = aux.clip(min=-1.0, max=1.0)

        # Publish message
        if(self.msg.setpoints[0]!=0.0 and self.msg.setpoints[0]!=0.0):
            self.msg.header.stamp = rospy.Time.now()
            self.pub_thrusters_setpoints.publish(self.msg)
            self.msg.header.seq = self.msg.header.seq + 1


    def check_map_ack(self, event):

        if self.map_ack_init:
            # rospy.loginfo("last_ack "+(str(rospy.Time.now().to_sec() - self.last_map_ack)))

            if self.map_ack_alive:
                self.map_ack_alive = False
            else:
                rospy.loginfo("%s: we have lost map_ack!")
                #set setpoints to 0
                self.msg.setpoints = [0.0,0.0]
                self.pub_thrusters_setpoints.publish(self.msg)
                print "map_ack lost"
        else:
            rospy.loginfo("%s: waiting for map ack...")

        # Send ack message
        msg = String()
        msg.data = str(self.seq) + ' ok'
        self.pub_check_joystick.publish(msg)



if __name__ == '__main__':

    rospy.init_node('Controller')
    Controller()
    rospy.spin()


