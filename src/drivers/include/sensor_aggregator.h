#ifndef SENSOR_AGGREGATOR_
#define SENSOR_AGGREGATOR_H

#include "ros/ros.h"

// Service includes
#include <std_srvs/Empty.h>

// Message includes
#include <sensor_msgs/Imu.h>
#include <cola2_xiroi/EnableNavStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// Package includes
#include "sensor.h"
#include <ned_tools/ned.h>

#include <boost/thread.hpp>

using namespace std;
using namespace boost;

typedef geometry_msgs::PoseWithCovarianceStamped PoseMsg;
typedef geometry_msgs::TwistWithCovarianceStamped TwistMsg;
typedef sensor_msgs::Imu ImuMsg;
typedef nav_msgs::Odometry OdomMsg;
typedef sensor_msgs::NavSatFix::ConstPtr GpsMsg;


class SensorAggregator {

 public:
  SensorAggregator();

 private:
  // Node Handles
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Timer timer_;
  std::string node_name_;

  // Sensors
  Sensor<ImuMsg, ImuMsg>* imu_;
  Sensor<GpsMsg, PoseMsg>* gps_;
  bool all_sensors_are_init_;

  // Altitude publisher
  ros::Publisher altitude_pub_;

  // GPS pose to EKF initial pose
  double north_;
  double east_;
  bool initial_robot_pose_is_set_;

  // Services
  ros::ServiceClient enable_nav_status_;

  // Covariances
  double gps_covariance_;


  // Safety
  bool enable_sensors_safety_;



  // Global messages needed
  ImuMsg imu_msg_;

  // Mutex
  mutex mutex_imu_;

  // Frames
  std::string base_frame_id_;
  std::string robot_frame_id_;
  std::string odom_frame_id_;
  std::string map_frame_id_;
  std::string origin_suffix_;

  // TF for set pose
  bool tf_catched_;
  tf::TransformListener tf_listener_;

  // GPS filter for set pose
  vector< pair<double, double> > gps_samples_;
  int gps_init_filter_size_;
  double gps_max_ned_dist_;

  // Static transforms
  tf::StampedTransform gps_tf_;

  // Safety
  int max_num_errors_;

  // Set pose topic names
  std::string set_pose_;
  ros::Publisher pose_pub_;


  // NED conversion library instance
  Ned* ned_;

  void updateImu(const ImuMsg& msg);
  void updateGps(const GpsMsg& msg);
  bool checkGpsRequiredVariables(double gps_stamp);
  void setPose(double north, double east, double depth, double north_cov, double east_cov, double depth_cov, ros::Time gps_stamp);
};

#endif // SENSOR_AGGREGATOR_H
