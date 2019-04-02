#include "include/sensor_aggregator.h"

SensorAggregator::SensorAggregator()
    : nhp_("~"), north_(0), east_(0), initial_robot_pose_is_set_(false), tf_catched_(false) {

  // Setup the node name
  node_name_ = ros::this_node::getName();

  // Read parameters
  // Get sensor covariances
  nh_.param("sensors/gps_covariance", gps_covariance_, 0.0);


  // Get sensor parameters
  nh_.param("sensors/gps_init_filter_size", gps_init_filter_size_, 8);
  nh_.param("sensors/gps_max_ned_dist", gps_max_ned_dist_, 500.0);

  // Frames
  nh_.param("frames/map", map_frame_id_, std::string(""));
  nh_.param("frames/base_link", base_frame_id_, std::string(""));
  nh_.param("frames/robot", robot_frame_id_, std::string(""));

  // Get set pose topic names
  nh_.param("sensors/set_pose", set_pose_, std::string("ekf_map/set_pose"));

  // Instantiate set of sensors
  imu_   = new Sensor<ImuMsg,   ImuMsg>  ("imu",   "sensors/imu_raw",      "sensors/imu"      );
  gps_   = new Sensor<GpsMsg,   PoseMsg> ("gps",   "sensors/gps_raw",      "sensors/gps"      );

  // Set up the set_pose publishers to set the poses of the ekf filters
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(set_pose_, 1);

  // Set up callbacks
  imu_->setCallback(boost::bind(&SensorAggregator::updateImu,this,_1));
  gps_->setCallback(boost::bind(&SensorAggregator::updateGps,this,_1));
}


void SensorAggregator::updateImu(const ImuMsg& msg) {
  // Store last imu message
  {
    mutex::scoped_lock lock(mutex_imu_);
    imu_msg_ = msg;
  }
  // Transform from NED to ENU to work with the Robot Localization Filter
  // double ax, ay, az, wx, wy, wz;
  // ax = msg.linear_acceleration.x;
  // ay = msg.linear_acceleration.y;
  // az = msg.linear_acceleration.z;
  // wx = msg.angular_velocity.x;
  // wy = msg.angular_velocity.y;
  // wz = msg.angular_velocity.z;
  // tf::Quaternion q(msg.orientation.x,
  //                  msg.orientation.y,
  //                  msg.orientation.z,
  //                  msg.orientation.w);
  // // Quaternion to Euler
  // tf::Matrix3x3 m(q);
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  // // Euler to Quaternion
  // tf::Quaternion q2;
  // q2.setEuler(-yaw, -pitch, roll);

  // ImuMsg imu_msg;
  // imu_msg.header.stamp = msg.header.stamp;
  // imu_msg.header.frame_id = msg.header.frame_id;
  // imu_msg.linear_acceleration.x = -ay;
  // imu_msg.linear_acceleration.y = ax;
  // imu_msg.linear_acceleration.z = -az;
  // imu_msg.angular_velocity.x = -wy;
  // imu_msg.angular_velocity.y = wx;
  // imu_msg.angular_velocity.z = -wz;
  // imu_msg.orientation.x = q2[1];
  // imu_msg.orientation.y = q2[0];
  // imu_msg.orientation.z = -q2[2];
  // imu_msg.orientation.w = q2[3];

  // imu_->publish(imu_msg);
}

void SensorAggregator::updateGps(const GpsMsg& msg) {
  // Get the NED origin to convert lat/lon to x/y
  double ned_latitude, ned_longitude;
  bool s1, s2;
  s1 = nh_.getParamCached("navigator/ned_latitude", ned_latitude);
  s2 = nh_.getParamCached("navigator/ned_longitude", ned_longitude);

  if (s1 && s2) {
    // Set up current NED origin, in case it changes
    ned_ = new Ned(ned_latitude, ned_longitude, 0.0);

    // Take current lat, lon and convert it to NED
    double latitude  = msg->latitude;
    double longitude = msg->longitude;
    double altitude = msg->altitude;
    double north, east, depth;
    ned_->geodetic2Ned(latitude, longitude, altitude, north, east, depth);

    // Security checks
    double dist_to_ned_origin = sqrt(north*north + east*east);
    if (dist_to_ned_origin > gps_max_ned_dist_) {
      ROS_ERROR_STREAM(node_name_ << ": Distance to NED origin too large: " << dist_to_ned_origin << "m (error at " << gps_max_ned_dist_ << "m).");
      return;
    } else if (dist_to_ned_origin > gps_max_ned_dist_*0.8) {
      ROS_WARN_STREAM(node_name_ << ": Distance to NED origin is large: " << dist_to_ned_origin << " m (warning at " << gps_max_ned_dist_*0.8 << "m).");
    }

    // Send the initial pose to the ekf
    if (!initial_robot_pose_is_set_) {
      // Filter the gps measures
      if (gps_samples_.size() < (uint)gps_init_filter_size_)
      {
        ROS_INFO_STREAM_THROTTLE(2, node_name_ << ": Initializing gps: adding samples to gps filter.");
        gps_samples_.push_back(make_pair(north, east)); //TODO: add mean depth
        return;
      }

      // Check the required variables to process gps
      if (!checkGpsRequiredVariables(msg->header.stamp.toSec()))
      {
        ROS_WARN_STREAM_THROTTLE(4, node_name_ << ": Waiting for required gps variables...");
        return;
      }

      ROS_INFO_STREAM(node_name_ << ": Initializing gps: done.");

      // Compute the mean
      double north_mean = 0.0;
      double east_mean  = 0.0;
      for (size_t i=0; i<gps_samples_.size(); i++) {
        north_mean += gps_samples_[i].first;
        east_mean  += gps_samples_[i].second;
      }
      north_mean = north_mean / gps_samples_.size();
      east_mean  = east_mean  / gps_samples_.size();

      // Clear the vector
      gps_samples_.clear();

      // Start the publication of sensor messages
      initial_robot_pose_is_set_ = true;

      // Set the pose
      setPose(north_mean, east_mean, depth, msg->position_covariance[0], msg->position_covariance[4], msg->position_covariance[8], msg->header.stamp);
    }

    // Create a new pose
    PoseMsg pose_msg;
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frame_id = msg->header.frame_id;
    pose_msg.pose.pose.position.x = north;
    pose_msg.pose.pose.position.y = east;
    // Do not use depth from geodetic2Ned as it is different from zero!
    pose_msg.pose.pose.position.z = 0.0;

    pose_msg.pose.covariance[0] = msg->position_covariance[0];
    pose_msg.pose.covariance[7] = msg->position_covariance[4];

    gps_->publish(pose_msg);
    
  } 
}


bool SensorAggregator::checkGpsRequiredVariables(double gps_stamp)
{
  // Get tf
  if (!tf_catched_) {
    // Catch static tfs
    try {
      tf_listener_.lookupTransform(base_frame_id_,
                                   robot_frame_id_ + "/" + gps_->getName(),
                                   ros::Time(0),
                                   gps_tf_);
    } catch (tf::TransformException ex) {
      ROS_WARN_STREAM(node_name_ << ": " << ex.what());
      return false;
    }
    tf_catched_ = true;
  }

  if (imu_->init() && tf_catched_) {

    mutex::scoped_lock lock1(mutex_imu_);

    double imu_stamp = imu_msg_.header.stamp.toSec();
    if ( fabs( imu_stamp - gps_stamp ) > 1.0 ) {
      ROS_WARN_STREAM_THROTTLE(1, node_name_ << ": Difference between gps stamp and imu stamp to big...");
      return false;
    }
    return true;
  }

  return false;
}

void SensorAggregator::setPose(double north, double east, double depth, double north_cov, double east_cov, double depth_cov, ros::Time gps_stamp)
{
  geometry_msgs::PoseWithCovarianceStamped pose_msg;

  ImuMsg imu_msg;
  {
    mutex::scoped_lock lock(mutex_imu_);
    imu_msg = imu_msg_;
  }

  tf::Quaternion q(imu_msg.orientation.x,
                   imu_msg.orientation.y,
                   imu_msg.orientation.z,
                   imu_msg.orientation.w);
  tf::Transform imu_orientation(q);

  tf::Quaternion dummy_q(0, 0, 0, 1);

  tf::Vector3 gps_point(north, east, depth);
  tf::Transform gps_meas(dummy_q, gps_point);


  // Vehicle orientation = vehicle2imu * imu orientation
  tf::Transform robot_orientation = imu_orientation;

  // updated_measure = rot * measure * tf_sensor2sparus
  gps_meas   =  gps_meas  * robot_orientation *   gps_tf_.inverse();

  // Set pose
  pose_msg.pose.pose.position.x = gps_meas.getOrigin().x();
  pose_msg.pose.pose.position.y = gps_meas.getOrigin().y();
  pose_msg.pose.pose.position.z = gps_meas.getOrigin().z();
  pose_msg.pose.pose.orientation.x = robot_orientation.getRotation().x();
  pose_msg.pose.pose.orientation.y = robot_orientation.getRotation().y();
  pose_msg.pose.pose.orientation.z = robot_orientation.getRotation().z();
  pose_msg.pose.pose.orientation.w = robot_orientation.getRotation().w();
  // Set the covariances
  pose_msg.pose.covariance[0]  = north_cov;
  pose_msg.pose.covariance[7]  = east_cov;
  pose_msg.pose.covariance[14] = depth_cov;
  pose_msg.pose.covariance[21] = imu_msg.orientation_covariance[0];
  pose_msg.pose.covariance[28] = imu_msg.orientation_covariance[4];
  pose_msg.pose.covariance[35] = imu_msg.orientation_covariance[8];

  // Set the stamp
  pose_msg.header.stamp = gps_stamp;

  // Send the message to the topic for ekf
  pose_msg.header.frame_id = map_frame_id_;
  pose_pub_.publish(pose_msg);

  // Enable the publication of nav_status
  enable_nav_status_ = nh_.serviceClient<cola2_xiroi::EnableNavStatus>("nav_status/enable");
  while (!enable_nav_status_.waitForExistence())
  {
    ROS_INFO_STREAM_ONCE(node_name_ << ": Waiting for nav_status/enable service to be available.");
  }
  cola2_xiroi::EnableNavStatus srv;
  srv.request.enable = true;
  enable_nav_status_.call(srv);

  // Log
  double r,p,y;
  robot_orientation.getBasis().getRPY(r, p, y);
  ROS_INFO_STREAM(node_name_ << ": /SetPose service called!"
    << "\n\t (X, Y, Z):  (" << pose_msg.pose.pose.position.x << ", "
                            << pose_msg.pose.pose.position.y << ", "
                            << pose_msg.pose.pose.position.z << ")"
    << "\n\t (R, P, Y):  (" << r << ", " << p << ", " << y << ")");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_aggregator");
  SensorAggregator s;
  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();
  return 0;
}