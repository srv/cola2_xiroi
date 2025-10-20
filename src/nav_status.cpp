
#include <ros/ros.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <cola2_msgs/NavSts.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <boost/filesystem.hpp>

#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib/utils/ned.h>
#include <cola2_xiroi/SetNedOrigin.h>
#include <cola2_xiroi/EnableNavStatus.h>
#include <cola2_lib_ros/this_node.h>

using namespace std;
namespace fs  = boost::filesystem;


class NavStatus
{
public:
    NavStatus(ros::NodeHandle nh);

protected:
    // Callbacks
    void odomCb(const nav_msgs::OdometryConstPtr& odom);
    void imuCb(const sensor_msgs::ImuConstPtr& imu);

    bool getNedOrigin(double& ned_latitude, double& ned_longitude);

    bool setNedOrigin(cola2_xiroi::SetNedOrigin::Request  &req,
                      cola2_xiroi::SetNedOrigin::Response &res);

    bool enable(cola2_xiroi::EnableNavStatus::Request  &req,
                cola2_xiroi::EnableNavStatus::Response &res);

private:
    ros::Publisher nav_pub_;
    ros::ServiceServer set_ned_origin_;
    ros::ServiceServer enable_nav_status_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;

    string node_name_;
    bool is_init_;
    bool enable_;
    bool imu_covariance_init_;
    ros::NodeHandle nh_;
    cola2::ros::DiagnosticHelper diagnostic_;
    double imu_r_cov_;
    double imu_p_cov_;
    double imu_y_cov_;
    string places_dir_;
    cola2::utils::NED ned_;
    double global_ned_lat_;
    double global_ned_lon_;
    double altitude_;
    double altitude_stamp_;
};

NavStatus::NavStatus(ros::NodeHandle nh): is_init_(false), enable_(false), imu_covariance_init_(false), nh_(nh), diagnostic_(nh, "/nav_status", ""), ned_(0.0, 0.0, 0.0)
{
    ros::NodeHandle nhp("~");
    node_name_ = ros::this_node::getName();

    // Init altitude
    altitude_stamp_ = -1.0;

    // Publishers
    nav_pub_ = nh.advertise<cola2_msgs::NavSts>("nav_sts", 1);

    // Services
    set_ned_origin_    = nhp.advertiseService("set_ned_origin", &NavStatus::setNedOrigin, this);
    enable_nav_status_ = nhp.advertiseService("enable", &NavStatus::enable, this);

    // Setup callbacks
    odom_sub_ = nh_.subscribe("odometry/filtered/global/ned", 1, &NavStatus::odomCb, this);
    imu_sub_ = nh_.subscribe("imu/data", 1, &NavStatus::imuCb, this);

    // Get the places directory
    if (nhp.hasParam("places_dir"))
        nhp.getParam("places_dir", places_dir_);
    else
        ROS_ERROR_STREAM("[" << node_name_ << "]: Impossible to get the places_dir param.");
}

void NavStatus::imuCb(const sensor_msgs::ImuConstPtr& imu)
{
    // Store
    imu_r_cov_ = imu->orientation_covariance[0];
    imu_p_cov_ = imu->orientation_covariance[4];
    imu_y_cov_ = imu->orientation_covariance[8];

    // Set as received
    imu_covariance_init_ = true;
    diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK, "Imu message received and initialized.");
    // diagnostic_.add("Imu Status", "Imu message received and initialized.");

    // Stop the callback
    imu_sub_.shutdown();
}

void NavStatus::odomCb(const nav_msgs::OdometryConstPtr& odom)
{
    // Exit until nav_status is enabled
    if (!enable_) return;

    // Wait until imu is initialized
    if (!imu_covariance_init_)
    {
        diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::WARN, "Not imu messages received!");
        // diagnostic_.add("Imu Status", "Not imu messages received!");
        return;
    }

    // Read the need origin from parameter server
    double ned_latitude, ned_longitude;
    if (!getNedOrigin(ned_latitude, ned_longitude))
    {
        ROS_ERROR_STREAM("[" << node_name_ << "]: Impossible to get the ned origin from the parameter server.");
        return;
    }
    ned_ = cola2::utils::NED(ned_latitude, ned_longitude, 0.0);
    // ned_ = new Ned(ned_latitude, ned_longitude, 0.0);
    // Build the nav status message
    cola2_msgs::NavSts nav;
    nav.header.stamp = odom->header.stamp;
    nav.header.frame_id = odom->header.frame_id;

    // Store ned origin
    nav.origin.latitude  = ned_.getInitLatitude();
    nav.origin.longitude = ned_.getInitLongitude();
    // nav.origin.latitude  = ned_->getLat();
    // nav.origin.longitude = ned_->getLon();

    // Store actual pose
    nav.position.north = odom->pose.pose.position.x;
    nav.position.east  = odom->pose.pose.position.y;
    nav.position.depth = odom->pose.pose.position.z;

    // Get global pose of actual pose
    double lat, lon, height;
    ned_.ned2Geodetic(nav.position.north, nav.position.east, nav.position.depth, lat, lon, height);
    // ned_->ned2Geodetic(nav.position.north, nav.position.east, nav.position.depth, lat, lon, height);
    nav.global_position.latitude  = lat;
    nav.global_position.longitude = lon;
    global_ned_lat_ = lat;
    global_ned_lon_ = lon;

    // Store velocity
    nav.body_velocity.x = odom->twist.twist.linear.x;
    nav.body_velocity.y = odom->twist.twist.linear.y;
    nav.body_velocity.z = odom->twist.twist.linear.z;

    // Get euler angles from quaternion
    tf::Quaternion q(odom->pose.pose.orientation.x,
                     odom->pose.pose.orientation.y,
                     odom->pose.pose.orientation.z,
                     odom->pose.pose.orientation.w);
    tf::Matrix3x3 rot(q);
    double roll, pitch, yaw;
    rot.getRPY(roll, pitch, yaw);
    nav.orientation.roll  = roll;
    nav.orientation.pitch = pitch;
    nav.orientation.yaw   = yaw;

    // Store rates
    nav.orientation_rate.roll  = odom->twist.twist.angular.x;
    nav.orientation_rate.pitch = odom->twist.twist.angular.y;
    nav.orientation_rate.yaw   = odom->twist.twist.angular.z;

    // Store altitude
    nav.altitude = altitude_;

    // Store covariances
    nav.position_variance.north    = odom->pose.covariance[0];
    nav.position_variance.east     = odom->pose.covariance[7];
    nav.position_variance.depth    = odom->pose.covariance[14];
    nav.orientation_variance.roll  = imu_r_cov_;
    nav.orientation_variance.pitch = imu_p_cov_;
    nav.orientation_variance.yaw   = imu_y_cov_;

    // Publish
    nav_pub_.publish(nav);
    is_init_ = true;
}

bool NavStatus::getNedOrigin(double& ned_latitude, double& ned_longitude)
{
    const string param_ned_latitude = "navigator/ned_latitude";
    const string param_ned_longitude = "navigator/ned_longitude";

    if (nh_.hasParam(param_ned_latitude) && nh_.hasParam(param_ned_longitude))
    {
        nh_.getParamCached(param_ned_latitude, ned_latitude);
        nh_.getParamCached(param_ned_longitude, ned_longitude);
        return true;
    }
    else
        return false;
}

bool NavStatus::setNedOrigin(cola2_xiroi::SetNedOrigin::Request  &req,
                             cola2_xiroi::SetNedOrigin::Response &res)
{
    if (!is_init_)
    {
        ROS_WARN_STREAM("[" << node_name_ << "]: NavStatus is not initialized yet. NED origin not saved!");
        res.success = "Nav status is not initialized yet...";
        return false;
    }

    string place_filename = req.place;
    if (place_filename != "")
    {
        // Sanity checks
        replace(place_filename.begin(), place_filename.end(), ' ', '_');
        fs::path tmp(place_filename);
        if (tmp.extension() == "")
            place_filename += ".yaml";

        // Build path
        fs::path file(place_filename);
        fs::path dir(places_dir_);
        fs::path full_path = dir / file;
        string fpath = full_path.string();

        // Check if file exists
        if (fs::exists(fpath))
        {
            ROS_WARN_STREAM("[" << node_name_ << "]: The file " << fpath << " already exists. NED origin not saved!");
            return false;
        }

        // Save
        fstream ofile(fpath.c_str(), ios::out | ios::trunc);
        ofile << "/navigator/ned_latitude: " << setprecision(6) << global_ned_lat_ << "\n";
        ofile << "/navigator/ned_longitude: " << setprecision(6) << global_ned_lon_;
        ofile.close();

        // Set to the ros param server
        nh_.setParam("/navigator/ned_latitude", global_ned_lat_);
        nh_.setParam("/navigator/ned_longitude", global_ned_lon_);

        ROS_INFO_STREAM("[" << node_name_ << "]: NED initialized with latitude / longitude: " << global_ned_lat_ << " / " << global_ned_lon_);
        ROS_INFO_STREAM("[" << node_name_ << "]: NED origin saved in: " << fpath);
        res.success = "NED initialized and saved.";
        return true;
    }
    else
    {
        res.success = "Ned origin not saved! the place filename is empty!";
        ROS_WARN_STREAM("[" << node_name_ << "]: Ned origin not saved! the place filename is empty!");
        return false;
    }
}

bool NavStatus::enable(cola2_xiroi::EnableNavStatus::Request  &req,
                       cola2_xiroi::EnableNavStatus::Response &res)
{
    bool enable = req.enable;
    if (enable)
    {
        ROS_INFO_STREAM("[" << node_name_ << "]: Publisher enabled.");
        res.success = "Nav_status enabled.";
        enable_ = true;
    }
    else
    {
        ROS_INFO_STREAM("[" << node_name_ << "]: Publisher disabled.");
        res.success = "Nav_status disabled.";
        enable_ = false;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_status");

    ros::NodeHandle nh;
    NavStatus node(nh);

    ros::spin();
    ros::shutdown();

    return 0;
}