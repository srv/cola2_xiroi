#! /usr/bin/env python
"""LogitechFX10 controler node."""

import rospy
from std_srvs.srv import Empty
from joystick_base import JoystickBase

class LogitechFX10(JoystickBase):
    """LogitechFX10 controler class."""

    """
        This class inherent from JoystickBase. It has to overload the
        method update_joy(self, joy) that receives a sensor_msgs/Joy
        message and fill the var self.joy_msg as described in the class
        JoystickBase.
        From this class it is also possible to call services or anything
        else reading the buttons in the update_joy method.
    """

    def __init__(self, name):
        """Class Constructor."""
        JoystickBase.__init__(self, name)

        # ... enable thrusters
        # rospy.wait_for_service(
        #     'control/enable_thrusters', 10)
        # try:
        #     self.enable_thrusters = rospy.ServiceProxy(
        #         'control/enable_thrusters', Empty)
        # except rospy.ServiceException, e:
        #     rospy.logwarn("%s: Service call failed: %s", self.name, e)

        # ... disable thrusters
        # rospy.wait_for_service(
        #     'control/disable_thrusters', 10)
        # try:
        #     self.disable_thrusters = rospy.ServiceProxy(
        #         'control/disable_thrusters', Empty)
        # except rospy.ServiceException, e:
        #     rospy.logwarn("%s: Service call failed: %s", self.name, e)

        # ... enable keep position
        # rospy.wait_for_service(
        #     'control/enable_keep_position', 10)
        # try:
        #     self.enable_keep_position = rospy.ServiceProxy(
        #         'control/enable_keep_position', Empty)
        # except rospy.ServiceException, e:
        #     rospy.logwarn("%s: Service call failed: %s", self.name, e)

         # ... disable keep position
        # rospy.wait_for_service(
        #     'control/disable_keep_position', 10)
        # try:
        #     self.disable_keep_position = rospy.ServiceProxy(
        #         'control/disable_keep_position', Empty)
        # except rospy.ServiceException, e:
        #     rospy.logwarn("%s: Service call failed: %s", self.name, e)

        # ... enable teleoperation
        # rospy.wait_for_service(
        #     'control/enable_teleoperation', 10)
        # try:
        #     self.enable_teleoperation = rospy.ServiceProxy(
        #         'control/enable_teleoperation', Empty)
        # except rospy.ServiceException, e:
        #     rospy.logwarn("%s: Service call failed: %s", self.name, e)
        # ... disable teleoperation
        # rospy.wait_for_service(
        #     'control/disable_teleoperation', 10)
        # try:
        #     self.disable_teleoperation = rospy.ServiceProxy(
        #         'control/disable_teleoperation', Empty)
        # except rospy.ServiceException, e:
        #     rospy.logwarn("%s: Service call failed: %s", self.name, e)


    def update_joy(self, joy):
        """Receive joy raw data."""
        """ Transform FX10 joy data into 12 axis data (pose + twist)
        and sets the buttons that especify if position or velocity
        commands are used in the teleoperation."""

        # JOYSTICK  DEFINITION:
        LEFT_JOY_HORIZONTAL = 0     # LEFT+, RIGHT-
        LEFT_JOY_VERTICAL = 1       # UP+, DOWN-
        LEFT_TRIGGER = 2            # NOT PRESS 1, PRESS -1
        RIGHT_JOY_HORIZONTAL = 3    # LEFT+, RIGHT-
        RIGHT_JOY_VERTICAL = 4      # UP+, DOWN-
        RIGHT_TRIGGER = 5           # NOT PRESS 1, PRESS -1
        CROSS_HORIZONTAL = 6        # LEFT+, RIGHT-
        CROSS_VERTICAL = 7          # UP+, DOWN-
        BUTTON_A = 0
        BUTTON_B = 1
        BUTTON_X = 2
        BUTTON_Y = 3
        BUTTON_LEFT = 4
        BUTTON_RIGHT = 5
        BUTTON_BACK = 6
        BUTTON_START = 7
        BUTTON_LOGITECH = 8
        BUTTON_LEFT_JOY = 9
        BUTTON_RIGHT_JOY = 10
        MOVE_UP = 1
        MOVE_DOWN = -1
        MOVE_LEFT = 1
        MOVE_RIGHT = -1

        self.joy_msg.header = joy.header

        self.joy_msg.axes[JoystickBase.AXIS_TWIST_U] = joy.axes[ LEFT_JOY_VERTICAL]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_V] = -joy.axes[RIGHT_JOY_HORIZONTAL]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_W] = joy.axes[RIGHT_JOY_VERTICAL]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_R] = joy.axes[LEFT_JOY_HORIZONTAL]

        # Enable/disable thrusters
        # if joy.axes[LEFT_TRIGGER] < -0.9 and joy.axes[RIGHT_TRIGGER] < -0.9:
        #     self.disable_thrusters()
        #     rospy.sleep(1.0)

        # if joy.buttons[BUTTON_LEFT] == 1.0 and joy.buttons[BUTTON_RIGHT] == 1.0:
        #     self.enable_thrusters()

        # Enable/disable keep position
        # if joy.buttons[BUTTON_START] == 1.0:
        #     self.enable_keep_position()

        # if joy.buttons[BUTTON_BACK] == 1.0:
        #     self.disable_keep_position()

        #Enable/disable teleoperation
        # if joy.buttons[BUTTON_A] == 1.0:
        #     self.enable_teleoperation()

        # if joy.buttons[BUTTON_Y] == 1.0:
        #     self.disable_teleoperation()

if __name__ == '__main__':
    """ Initialize the logitech_fx10 node. """
    try:
        rospy.init_node('logitech_fx10')
        map_ack = LogitechFX10(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass