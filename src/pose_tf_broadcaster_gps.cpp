#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>

#include "cola2_lib/rosutils/diagnostic_helper.h"
#include "ned_tools/ned.h"

using namespace std;

class PoseTfBroadcaster
{
  public:
    PoseTfBroadcaster(ros::NodeHandle nh);

  protected:
    void gpsCb(const sensor_msgs::NavSatFixConstPtr &gps);
    bool getNedOrigin(double& ned_origin_lat, double& ned_origin_lon);

  private:
    ros::Subscriber gps_sub_;

    // Operational variables
    tf::TransformBroadcaster tf_broadcaster_;
    Ned* ned_;

    // Node parameters
    string map_frame_id_;
    string gps_frame_id_;

    string node_name_;
    ros::NodeHandle nh_;

};

PoseTfBroadcaster::PoseTfBroadcaster(ros::NodeHandle nh): nh_(nh)
{   // Node name
    node_name_ = ros::this_node::getName();

    // GPS callback
    gps_sub_ = nh.subscribe("gps", 10, &PoseTfBroadcaster::gpsCb, this);

    // Get params
    nh.param("/xiroi/frames/map", map_frame_id_, string("world"));
    nh.param("/xiroi/frames/gps_usbl", gps_frame_id_, string("gps_usbl"));
}

void PoseTfBroadcaster::gpsCb(const sensor_msgs::NavSatFix::ConstPtr &gps)
{
    // NED
    double ned_origin_lat, ned_origin_lon;
    if (!getNedOrigin(ned_origin_lat, ned_origin_lon))
    {
        ROS_ERROR_STREAM("[" << node_name_ << "]: Impossible to get the ned origin from the parameter server.");
    }
    ned_ = new Ned(ned_origin_lat, ned_origin_lon, 0.0);
    double north, east, depth;
    ned_->geodetic2Ned(gps->latitude, gps->longitude, 0.0, north, east, depth);

    // Publish  gps TF
    tf::Transform tf_gps;
    tf::Vector3 tf_gps_v(north, east, 0.0);
    tf::Quaternion tf_gps_q(0.0, 0.0, 0.0, 1);
    tf_gps.setOrigin(tf_gps_v);
    tf_gps.setRotation(tf_gps_q);
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_gps, gps->header.stamp, map_frame_id_, gps_frame_id_));
}


bool PoseTfBroadcaster::getNedOrigin(double& ned_origin_lat, double& ned_origin_lon)
{
    const string param_ned_origin_lat = "/xiroi/navigator/ned_latitude";
    const string param_ned_origin_lon = "/xiroi/navigator/ned_longitude";

    if (nh_.hasParam(param_ned_origin_lat) && nh_.hasParam(param_ned_origin_lon))
    {
        nh_.getParamCached(param_ned_origin_lat, ned_origin_lat);
        nh_.getParamCached(param_ned_origin_lon, ned_origin_lon);
        return true;
    }
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_tf_broadcaster_gps");

    ros::NodeHandle nh;
    PoseTfBroadcaster node(nh);

    ros::spin();

    ros::shutdown();

    return 0;
}