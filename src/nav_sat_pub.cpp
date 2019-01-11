#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_sat_pub");

  ros::NodeHandle n;
  ros::NodeHandle np("~");

  string frame_id;
  string topic_name;
  double lat;
  double lon;
  bool use_ned_origin;
  np.param("frame_id", frame_id, string(""));
  np.param("topic_name", topic_name, string(""));
  np.param("use_ned_origin", use_ned_origin, false);
  np.param("lat", lat, 0.0);
  np.param("lon", lon, 0.0);

  ros::Publisher pub1 = n.advertise<sensor_msgs::NavSatFix>(topic_name, 1);
  ros::Rate loop_rate(5);

  // Publish the gps over the ned origin
  bool ned_catched = false;

  while (ros::ok())
  {
    if (!ned_catched && use_ned_origin)
    {
      if (n.hasParam("/xiroi/navigator/ned_latitude") && n.hasParam("/xiroi/navigator/ned_longitude"))
      {
        n.getParam("/xiroi/navigator/ned_latitude", lat);
        n.getParam("/xiroi/navigator/ned_longitude", lon);
        ned_catched = true;
      }
      else
      {
        ROS_WARN_THROTTLE(5, "[nav_sat_pub]: Unable to find /xiroi/navigator/ned_origin");
        continue;
      }
    }
    sensor_msgs::NavSatFix nav;
    nav.header.stamp = ros::Time::now();
    nav.header.frame_id = frame_id;
    nav.latitude = lat;
    nav.longitude = lon;
    pub1.publish(nav);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();

  return 0;
}