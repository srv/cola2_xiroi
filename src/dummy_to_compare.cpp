#include <ros/ros.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <cola2_msgs/NavSts.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <boost/filesystem.hpp>

#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib/utils/ned.h>
#include <cola2_xiroi/SetNedOrigin.h>
#include <cola2_xiroi/EnableNavStatus.h>
#include <cola2_lib_ros/this_node.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>

using namespace std;
namespace fs  = boost::filesystem;

class NavStatus
{
public:
    NavStatus(ros::NodeHandle nh);

protected:
    void odomCb(const nav_msgs::OdometryConstPtr& odom);
    void imuCb(const sensor_msgs::ImuConstPtr& imu);

    bool getNedOrigin(double& ned_latitude, double& ned_longitude);
    bool setNedOrigin(cola2_xiroi::SetNedOrigin::Request  &req,
                      cola2_xiroi::SetNedOrigin::Response &res);
    bool enable(cola2_xiroi::EnableNavStatus::Request  &req,
                cola2_xiroi::EnableNavStatus::Response &res);
    void publishDiagnostics();

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
    double last_imu_time_;
    double last_odom_time_;
    double last_gps_time_;  
    double last_dvl_time_;    
    double last_altitude_time_;
    double diagnostics_last_pub_time_;
};

NavStatus::NavStatus(ros::NodeHandle nh)
: is_init_(false), enable_(false), imu_covariance_init_(false), nh_(nh), 
  diagnostic_(nh, "/nav_status", ""), ned_(0.0, 0.0, 0.0)
{
    ros::NodeHandle nhp("~");
    node_name_ = ros::this_node::getName();

    last_imu_time_ = 0.0;
    last_odom_time_ = 0.0;
    last_gps_time_ = 0.0;
    last_dvl_time_ = 0.0;
    last_altitude_time_ = 0.0;
    diagnostics_last_pub_time_ = 0.0;

    // Init altitude
    altitude_stamp_ = -1.0;

    // Publishers
    nav_pub_ = nh.advertise<cola2_msgs::NavSts>("nav_sts", 1);

    // Services
    set_ned_origin_    = nhp.advertiseService("set_ned_origin", &NavStatus::setNedOrigin, this);
    enable_nav_status_ = nhp.advertiseService("enable", &NavStatus::enable, this);

    // Callbacks
    odom_sub_ = nh_.subscribe("odometry", 1, &NavStatus::odomCb, this);
    imu_sub_ = nh_.subscribe("imu/data", 1, &NavStatus::imuCb, this);

    // Places dir
    if (nhp.hasParam("places_dir"))
        nhp.getParam("places_dir", places_dir_);
    else
        ROS_ERROR_STREAM("[" << node_name_ << "]: Impossible to get the places_dir param.");

    // Timer para publicar diagnostics cada segundo
    ros::Timer diag_timer = nh_.createTimer(ros::Duration(1.0),
                                            [this](const ros::TimerEvent&){
                                                this->publishDiagnostics();
                                            });

    ROS_INFO_STREAM("[NavStatus] Timer for diagnostics created");
}

void NavStatus::imuCb(const sensor_msgs::ImuConstPtr& imu)
{
    ROS_INFO_STREAM("[NavStatus] imuCb called");
    
    imu_r_cov_ = imu->orientation_covariance[0];
    imu_p_cov_ = imu->orientation_covariance[4];
    imu_y_cov_ = imu->orientation_covariance[8];
    imu_covariance_init_ = true;

    last_imu_time_ = ros::Time::now().toSec();

    diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK, "IMU message received and initialized.");
    
    ROS_INFO_STREAM("[NavStatus] IMU initialized");
    
    imu_sub_.shutdown();
}

void NavStatus::odomCb(const nav_msgs::OdometryConstPtr& odom)
{
    ROS_DEBUG("[NavStatus] odomCb called");
    last_odom_time_ = ros::Time::now().toSec();

    if (!enable_)
    { 
        ROS_INFO_STREAM("[NavStatus] odomCb ignored: enable_ = false");
        return;
    }
    if (!imu_covariance_init_)
    {
        ROS_INFO_STREAM("[NavStatus] odomCb ignored: imu not initialized");
        diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::WARN, "Waiting for IMU messages!");
        return;
    }

    ROS_INFO_THROTTLE(5.0, "[NavStatus] odomCb processing odometry");

    double ned_latitude, ned_longitude;
    if (!getNedOrigin(ned_latitude, ned_longitude))
    {
        ROS_ERROR_STREAM("[" << node_name_ << "]: Impossible to get the NED origin from the parameter server.");
        return;
    }

    ned_ = cola2::utils::NED(ned_latitude, ned_longitude, 0.0);

    cola2_msgs::NavSts nav;
    nav.header.stamp = odom->header.stamp;
    nav.header.frame_id = odom->header.frame_id;

    nav.origin.latitude  = ned_.getInitLatitude();
    nav.origin.longitude = ned_.getInitLongitude();

    nav.position.north = odom->pose.pose.position.x;
    nav.position.east  = odom->pose.pose.position.y;
    nav.position.depth = odom->pose.pose.position.z;

    double lat, lon, height;
    ned_.ned2Geodetic(nav.position.north, nav.position.east, nav.position.depth, lat, lon, height);
    nav.global_position.latitude  = lat;
    nav.global_position.longitude = lon;
    global_ned_lat_ = lat;
    global_ned_lon_ = lon;

    nav.body_velocity.x = odom->twist.twist.linear.x;
    nav.body_velocity.y = odom->twist.twist.linear.y;
    nav.body_velocity.z = odom->twist.twist.linear.z;

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

    nav.orientation_rate.roll  = odom->twist.twist.angular.x;
    nav.orientation_rate.pitch = odom->twist.twist.angular.y;
    nav.orientation_rate.yaw   = odom->twist.twist.angular.z;

    nav.altitude = altitude_;

    nav.position_variance.north    = odom->pose.covariance[0];
    nav.position_variance.east     = odom->pose.covariance[7];
    nav.position_variance.depth    = odom->pose.covariance[14];
    nav.orientation_variance.roll  = imu_r_cov_;
    nav.orientation_variance.pitch = imu_p_cov_;
    nav.orientation_variance.yaw   = imu_y_cov_;

    nav_pub_.publish(nav);
    is_init_ = true;
}

void NavStatus::publishDiagnostics()
{
    // Construir DiagnosticArray
    diagnostic_msgs::DiagnosticArray arr;
    arr.header.stamp = ros::Time::now();

    // Helper lambda para crear KeyValue
    auto kv = [](const std::string &k, const std::string &v){
        diagnostic_msgs::KeyValue kv; kv.key = k; kv.value = v; return kv;
    };

    // 1) /navigation (estado general)
    diagnostic_msgs::DiagnosticStatus nav;
    nav.level = (is_init_ ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN);
    nav.name = "/navigation";
    nav.message = (is_init_ ? "OK" : "Not initialized");
    nav.hardware_id = ""; // como en tu ejemplo
    // añadir keys estándar (coincidir con estructuras antiguas)
    nav.values.push_back(kv("enabled", enable_ ? "true" : "false"));
    nav.values.push_back(kv("ned_init", (ned_.getInitLatitude() != 0.0 || ned_.getInitLongitude() != 0.0) ? "true" : "false"));
    // puedes calcular data_age de forma aproximada:
    double now = ros::Time::now().toSec();
    double imu_age = (last_imu_time_ == 0.0) ? 9999.0 : now - last_imu_time_;
    nav.values.push_back(kv("last_imu_data", std::to_string(imu_age)));
    // frecuencia vacía por ahora (si puedes calcular, ponla)
    nav.values.push_back(kv("frequency", "0.0"));
    arr.status.push_back(nav);

    // 2) /navigation/imu
    diagnostic_msgs::DiagnosticStatus imu_status;
    imu_status.level = (imu_covariance_init_ ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN);
    imu_status.name = "/navigation/imu";
    imu_status.hardware_id = node_name_;
    imu_status.message = imu_covariance_init_ ? "Ok" : "Waiting";
    imu_status.values.push_back(kv("enabled", imu_covariance_init_ ? "true" : "false"));
    imu_status.values.push_back(kv("data_age", std::to_string(imu_age)));
    imu_status.values.push_back(kv("frequency", "0.0"));
    arr.status.push_back(imu_status);

    // 3) /navigation/gps  (si no lo tienes, le ponemos WARN/disabled)
    diagnostic_msgs::DiagnosticStatus gps_status;
    gps_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    gps_status.name = "/navigation/gps";
    gps_status.hardware_id = node_name_;
    gps_status.message = "No GPS (or not used)";
    gps_status.values.push_back(kv("enabled", "false"));
    gps_status.values.push_back(kv("data_age", std::to_string((last_gps_time_==0.0)?9999.0:now-last_gps_time_)));
    gps_status.values.push_back(kv("frequency", "0.0"));
    arr.status.push_back(gps_status);

    // 4) /navigation/dvl
    diagnostic_msgs::DiagnosticStatus dvl_status;
    dvl_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    dvl_status.name = "/navigation/dvl";
    dvl_status.hardware_id = node_name_;
    dvl_status.message = "No DVL (or not used)";
    dvl_status.values.push_back(kv("enabled", "false"));
    dvl_status.values.push_back(kv("data_age", std::to_string((last_dvl_time_==0.0)?9999.0:now-last_dvl_time_)));
    dvl_status.values.push_back(kv("frequency", "0.0"));
    arr.status.push_back(dvl_status);

    // 5) /navigation/pressure (depth)
    diagnostic_msgs::DiagnosticStatus pressure_status;
    pressure_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    pressure_status.name = "/navigation/pressure";
    pressure_status.hardware_id = node_name_;
    pressure_status.message = "No pressure/depth sensor";
    pressure_status.values.push_back(kv("enabled", "false"));
    pressure_status.values.push_back(kv("data_age", std::to_string((last_altitude_time_==0.0)?9999.0:now-last_altitude_time_)));
    pressure_status.values.push_back(kv("frequency", "0.0"));
    arr.status.push_back(pressure_status);

    // 6) /navigation/navigator  (este suele llevar filter_init, ned_init, etc.)
    diagnostic_msgs::DiagnosticStatus navigator_status;
    navigator_status.level = (is_init_ ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN);
    navigator_status.name = "/navigation/navigator";
    navigator_status.hardware_id = "navigator";
    navigator_status.message = (is_init_ ? "Ok" : "Not initialized");
    navigator_status.values.push_back(kv("filter_init", is_init_ ? "true" : "false"));
    navigator_status.values.push_back(kv("ned_init", (ned_.getInitLatitude() != 0.0 || ned_.getInitLongitude() != 0.0) ? "true" : "false"));
    navigator_status.values.push_back(kv("gps_init", "false"));
    navigator_status.values.push_back(kv("dvl_init", "false"));
    navigator_status.values.push_back(kv("imu_init", imu_covariance_init_ ? "true" : "false"));
    navigator_status.values.push_back(kv("enabled", enable_ ? "true" : "false"));
    navigator_status.values.push_back(kv("data_age", "0.0"));
    navigator_status.values.push_back(kv("frequency", "0.0"));
    arr.status.push_back(navigator_status);

    // Publicar el array en el mismo topic que usa diagnostic_aggregator (normalmente /xiroi/diagnostics o /diagnostics)
    // diagnostic_aggregator suele suscribirse a '/xiroi/diagnostics' o '/diagnostics' dependiendo de namespace.
    // Usaremos el DiagnosticHelper si permite publicar el array; si no, creamos un publisher directo.
    // Aquí intentamos publicar con diagnostic_ (si tiene publisher), sino crear publisher local (defensivo).

    // Si DiagnosticHelper provee un publisher público (no todas las versiones lo hacen):
    // diagnostic_.publishArray(arr); // <-- si exists (NO sabemos si tu version tiene esto)
    // fallback: crear un publisher estático (localmente)
    static ros::Publisher raw_diag_pub = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    raw_diag_pub.publish(arr);

    ROS_DEBUG("[NavStatus] publishDiagnostics published %zu status entries", arr.status.size());
    diagnostics_last_pub_time_ = now;
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

bool NavStatus::enable(cola2_xiroi::EnableNavStatus::Request  &req,
                       cola2_xiroi::EnableNavStatus::Response &res)
{
    enable_ = req.enable;
    res.success = enable_ ? "NavStatus enabled" : "NavStatus disabled";
    ROS_INFO_STREAM("[" << node_name_ << "]: " << res.success);
    return true;
}

bool NavStatus::setNedOrigin(cola2_xiroi::SetNedOrigin::Request  &req,
                             cola2_xiroi::SetNedOrigin::Response &res)
{
    if (!is_init_)
    {
        res.success = "NavStatus is not initialized yet";
        ROS_WARN_STREAM("[" << node_name_ << "]: NED origin not saved!");
        return false;
    }

    string place_filename = req.place;
    if (place_filename != "")
    {
        replace(place_filename.begin(), place_filename.end(), ' ', '_');
        fs::path tmp(place_filename);
        if (tmp.extension() == "")
            place_filename += ".yaml";

        fs::path file(place_filename);
        fs::path dir(places_dir_);
        fs::path full_path = dir / file;
        string fpath = full_path.string();

        if (fs::exists(fpath))
        {
            ROS_WARN_STREAM("[" << node_name_ << "]: The file " << fpath << " already exists.");
            return false;
        }

        fstream ofile(fpath.c_str(), ios::out | ios::trunc);
        ofile << "/navigator/ned_latitude: " << setprecision(6) << global_ned_lat_ << "\n";
        ofile << "/navigator/ned_longitude: " << setprecision(6) << global_ned_lon_;
        ofile.close();

        nh_.setParam("/navigator/ned_latitude", global_ned_lat_);
        nh_.setParam("/navigator/ned_longitude", global_ned_lon_);

        ROS_INFO_STREAM("[" << node_name_ << "]: NED origin saved in: " << fpath);
        res.success = "NED initialized and saved.";
        return true;
    }
    else
    {
        res.success = "Place filename empty, NED not saved!";
        ROS_WARN_STREAM("[" << node_name_ << "]: Place filename empty!");
        return false;
    }
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