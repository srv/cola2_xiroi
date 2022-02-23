#!/usr/bin/python

import datetime
import rospy

import pytz


if __name__ == "__main__":
    rospy.init_node("tmp")
    tz = pytz.timezone('Europe/Madrid')

    now = datetime.datetime.now(tz)


    print("Datetime", now)

    now_ros = rospy.get_time()
    print("ROS", now_ros)
    rospy.spin()
