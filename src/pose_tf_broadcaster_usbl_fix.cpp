#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "cola2_lib/rosutils/diagnostic_helper.h"

using namespace std;

class PoseTfBroadcaster
{
  public:
    PoseTfBroadcaster(ros::NodeHandle nh);

  private:
    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent&);

    // Operational variables
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    // Node parameters
    string map_frame_id_;
    string usbl_frame_id_;
    string usbl_fix_frame_id_;

    string node_name_;
    ros::NodeHandle nh_;
    tf::StampedTransform tf_usbl_;

};

PoseTfBroadcaster::PoseTfBroadcaster(ros::NodeHandle nh): nh_(nh)
{   // Node name
    node_name_ = ros::this_node::getName();

    // Timer
    timer_ = nh_.createTimer(ros::Duration(0.05), &PoseTfBroadcaster::timerCallback,this);

    // Get params
    nh.param("/xiroi/frames/map", map_frame_id_, string("world_ned"));
    nh.param("/xiroi/frames/usbl", usbl_frame_id_, string("xiroi/base_link"));
    nh.param("/xiroi/frames/usbl_fix", usbl_fix_frame_id_, string("xiroi/usbl_fix"));
}

void PoseTfBroadcaster::timerCallback(const ros::TimerEvent&)
{
    // Listen USBL transform
    try{
      tf_listener_.lookupTransform(map_frame_id_, usbl_frame_id_, ros::Time(0), tf_usbl_);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.5).sleep();
    }
    // Publish  usbl_fix TF
    tf::Transform tf_usbl_fix;
    tf::Vector3 tf_usbl_fix_v(tf_usbl_.getOrigin().x(), tf_usbl_.getOrigin().y(), 0.0);
    tf::Quaternion tf_usbl_fix_q(0.0, 0.0, 0.0, 1);
    tf_usbl_fix.setOrigin(tf_usbl_fix_v);
    tf_usbl_fix.setRotation(tf_usbl_fix_q);
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_usbl_fix, ros::Time::now(), map_frame_id_, usbl_fix_frame_id_));
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_tf_broadcaster_usbl_fix");

    ros::NodeHandle nh;
    PoseTfBroadcaster node(nh);

    ros::spin();

    ros::shutdown();

    return 0;
}