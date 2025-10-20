#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
import tf.transformations as tft

class ENU2NEDBridge:
    def __init__(self):
        self.sub_odom_enu = rospy.Subscriber(
            '/xiroi/odometry/filtered/global/enu', Odometry, self.odom_callback)

        self.pub_odom_ned = rospy.Publisher(
            '/xiroi/odometry/filtered/global/ned', Odometry, queue_size=10
        )

        self.R_enu2ned = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])

    def odom_callback(self, msg_enu):
        msg_ned = Odometry()
        msg_ned.header = msg_enu.header
        msg_ned.header.frame_id = "map"  

        # --- Position ---
        pos_enu = np.array([msg_enu.pose.pose.position.x,
                            msg_enu.pose.pose.position.y,
                            msg_enu.pose.pose.position.z])
        pos_ned = self.R_enu2ned.dot(pos_enu)
        msg_ned.pose.pose.position.x = pos_ned[0]
        msg_ned.pose.pose.position.y = pos_ned[1]
        msg_ned.pose.pose.position.z = pos_ned[2]

        # --- Orientation ---
        q_enu = [msg_enu.pose.pose.orientation.x,
                 msg_enu.pose.pose.orientation.y,
                 msg_enu.pose.pose.orientation.z,
                 msg_enu.pose.pose.orientation.w]
        R_enu = tft.quaternion_matrix(q_enu)[:3, :3]
        R_ned = self.R_enu2ned.dot(R_enu)
        q_ned = tft.quaternion_from_matrix(np.vstack((
            np.hstack((R_ned, np.array([[0],[0],[0]]))),
            np.array([0,0,0,1])
        )))
        msg_ned.pose.pose.orientation.x = q_ned[0]
        msg_ned.pose.pose.orientation.y = q_ned[1]
        msg_ned.pose.pose.orientation.z = q_ned[2]
        msg_ned.pose.pose.orientation.w = q_ned[3]

        # --- Lineal Velocity ---
        v_enu = np.array([msg_enu.twist.twist.linear.x,
                          msg_enu.twist.twist.linear.y,
                          msg_enu.twist.twist.linear.z])
        v_ned = self.R_enu2ned.dot(v_enu)
        msg_ned.twist.twist.linear.x = v_ned[0]
        msg_ned.twist.twist.linear.y = v_ned[1]
        msg_ned.twist.twist.linear.z = v_ned[2]

        # --- Angular Velocity ---
        w_enu = np.array([msg_enu.twist.twist.angular.x,
                          msg_enu.twist.twist.angular.y,
                          msg_enu.twist.twist.angular.z])
        w_ned = self.R_enu2ned.dot(w_enu)
        msg_ned.twist.twist.angular.x = w_ned[0]
        msg_ned.twist.twist.angular.y = w_ned[1]
        msg_ned.twist.twist.angular.z = w_ned[2]

        # --- Covarianzas (opcional, si quieres ser riguroso) ---
        # Aquí también se podrían transformar las matrices de covarianza
        msg_ned.pose.covariance = msg_enu.pose.covariance
        msg_ned.twist.covariance = msg_enu.twist.covariance

        # Publish
        self.pub_odom_ned.publish(msg_ned)
        rospy.loginfo_throttle(2.0, f"Published NED odom: pos={pos_ned}, vel={v_ned}, w={w_ned}")

if __name__ == "__main__":
    rospy.init_node('enu_to_ned_bridge')
    bridge = ENU2NEDBridge()
    rospy.spin()