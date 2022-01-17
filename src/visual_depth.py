#!/usr/bin/python

import rospy
import tf2_ros
from sensor_msgs.msg import Range
from geometry_msgs.msg import TransformStamped


class VisualDepth:

    def __init__(self):

        BASE_LINK_FRAME_ID = rospy.get_param("~base_link_frame_id", default = "base_link")
        SENSOR_FRAME_ID = rospy.get_param("~sensor_frame_id", default = "camera")
        self.OFFSET = rospy.get_param('~base_link_to_water_surface', default = 0.1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer)

        try:
            self.transform = self.tf_buffer.lookup_transform(BASE_LINK_FRAME_ID, SENSOR_FRAME_ID, rospy.Time(0), rospy.Duration(10))

        except:
            rospy.logwarn("The tf from '%s' to '%s' does not seem to be available, will assume it as identity!", BASE_LINK_FRAME_ID, SENSOR_FRAME_ID)

            self.transform = TransformStamped()
            self.transform.header.stamp = rospy.Time.now()
            self.transform.header.frame_id = "xiroi/base_link"
            self.transform.child_frame_id = "xiroi/stereo_ch3/left_optical"
            self.transform.transform.translation.x = 0.0
            self.transform.transform.translation.y = 0.0
            self.transform.transform.translation.z = 0.0
            self.transform.transform.rotation.x = 0.0
            self.transform.transform.rotation.y = 0.0
            self.transform.transform.rotation.z = 0.0
            self.transform.transform.rotation.w = 1.0


        rospy.Subscriber("/xiroi/visual_altimeter/altitude", Range, self.callback)

        self.pub = rospy.Publisher('depth', Range, queue_size = 10)

    
    def callback(self, range_msg):

        if range_msg.range > 0.0:
            print(self.transform.transform.translation.z + range_msg.range)
            depth = self.transform.transform.translation.z + range_msg.range - self.OFFSET

        else:
            depth = range_msg.range

        depth_pub = Range()
        depth_pub.header.stamp = rospy.Time.now()
        depth_pub.range = depth
        self.pub.publish(depth_pub)


if __name__ == '__main__':
    rospy.init_node('visual_depth')
    visual_depth = VisualDepth()
    rospy.spin()
