
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/*@@>Takes the data from all navigation sensors (real or simulated) and merges
 them through a constant velocity model using an Extended Kalman Filter.<@@*/


#include <string>
#include "ros/ros.h"
#include <cola2_lib/cola2_navigation/EkfRosBase.h>
#include <cola2_lib/cola2_navigation/EkfSlamAuv.h>
#include <cola2_lib/cola2_navigation/nav_utils.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/Empty.h"
#include "cola2_msgs/TeledyneExplorerDvl.h"
#include "cola2_msgs/ValeportSoundVelocity.h"
#include "cola2_msgs/FastraxIt500Gps.h"
#include "cola2_msgs/PressureSensor.h"
#include "auv_msgs/BodyForceReq.h"
#include "cola2_msgs/Detection.h"
#include "cola2_msgs/RangeDetection.h"
#include "cola2_msgs/AddLandmark.h"
#include "cola2_msgs/Action.h"

#define USE_MODEL           false // Use force to velocity model to simulate DVL data if no botton and water velocity availables
#define INVALID_ALTITUDE    -3.2665

class S2RosNavigator: public EkfRosBase
{
public:
    S2RosNavigator( std::string name,
                    Eigen::VectorXd p_var ):
        EkfRosBase( name, true ),
        _p_var( p_var ),
        _dvl_init( false ),
        _pressure_init( false ),
        _imu_init( false ),
        _gps_init( false ),
        _ekf_init( false ),
        _ned_init( false ),
        _ned_error(false),
        _sound_velocity( 1500.0 ),
        _bottom_status( 0 ),
        _gps_samples( 0 ),
        _wrong_gps_samples_before_init(0),
        _echosounder_range( -1.0 ),
        _use_usbl_data( false ),
        _gps_samples_to_init(5),
        _declination( 0.0 ),
        _surface_to_depth_sensor_distance( 0.0 ),
        _depth_sensor_offset( 0.0 ),
        _last_pressure( 0.0 ),
        _initialize_depth_sensor_offset( false )
    {
        // Init subscribers
        _sub_explorer_dvl = _n.subscribe( "/cola2_navigation/teledyne_explorer_dvl", 2, &S2RosNavigator::updateExplorerDvl, this );
        _sub_pressure = _n.subscribe( "/cola2_navigation/pressure_sensor", 2, &S2RosNavigator::updatePressure, this );
        _sub_echosounder = _n.subscribe( "/airmar_echosounder", 2, &S2RosNavigator::updateEchosounder, this );
        _sub_imu = _n.subscribe( "/cola2_navigation/imu", 2, &S2RosNavigator::updateImu, this );
        _sub_gps = _n.subscribe( "/cola2_navigation/fastrax_it_500_gps", 2, &S2RosNavigator::updateGps, this );
        _sub_usbl = _n.subscribe( "/cola2_navigation/usbl_update", 2, &S2RosNavigator::updateUSBLEvologics, this );
        _sub_mb = _n.subscribe("/multibeam_range",2,&S2RosNavigator::updateMb, this);
        _sub_detection = _n.subscribe( "/cola2_navigation/detection_update", 2, &S2RosNavigator::updateLandmark, this );
        _sub_range = _n.subscribe("/cola2_navigation/range_update", 2, &S2RosNavigator::updateRange, this);
        _sub_world_fix = _n.subscribe("/cola2_navigation/world_fix_update", 2, &S2RosNavigator::updateWorldFixUpdate, this);

        if( USE_MODEL ) {
            _sub_model = _n.subscribe( "/cola2_control/merged_body_force_req", 2, &S2RosNavigator::updateBodyForceReq, this );
        }

        // Init services
        _reset_navigation_srv = _n.advertiseService("/cola2_navigation/reset_navigation", &S2RosNavigator::resetNavigation, this );
        _set_depth_sensor_offset_srv = _n.advertiseService("/cola2_navigation/set_depth_sensor_offset", &S2RosNavigator::setDepthSensorOffset, this );
        _add_landmark_srv = _n.advertiseService("/cola2_navigation/add_landmark", &S2RosNavigator::addLandmarkSrv, this );
        _reset_nav_action_srv = _n.advertiseService("/cola2_navigation/reset_navigation_action", &S2RosNavigator::resetNavigationAction, this );
        _reset_landmarks_srv = _n.advertiseService("/cola2_navigation/reset_landmarks", &S2RosNavigator::resetLandmarks, this );

        // Init timer
        _timer = _n.createTimer( ros::Duration(1.0), &S2RosNavigator::checkNavigatorDiagnostics, this );

        // Load configurations
        getConfig();
        _bar_to_meters_depth = 100000.0 / ( _water_density * 9.81 );

        assert( _model_covariance.size() == 3 );
        _q_var = Eigen::MatrixXd::Zero(3, 1);
        _q_var << _model_covariance.at(0), _model_covariance.at(1), _model_covariance.at(2);

        // Check NED and GPS configuration
        if( !_initialize_ned_from_gps && !_initialize_filter_from_gps && !_use_gps_data ) {
            _gps_init = true;
        }

        if( _initialize_ned_from_gps && !_initialize_filter_from_gps && _use_gps_data ) {
            std::cerr << "Weird configuration: Initialize NED from GPS and use GPS data but do not initialize filter from GPS\n";
        }
        else if( _initialize_ned_from_gps && !_initialize_filter_from_gps && !_use_gps_data ) {
            std::cerr << "Weird configuration: Initialize NED from GPS but do not initialize filter neither use data from GPS\n";
        }
        else if( !_initialize_ned_from_gps && !_initialize_filter_from_gps && _use_gps_data ) {
            std::cerr << "Invalid configuration: Do not initialize NED and filter from GPS but use data from it\n";
            std::cerr << "Use GPS data will be set to false\n";
            _use_gps_data = false;
        }

        // Init EKF
        initFilter();
    }


    void
    checkNavigatorDiagnostics( const ros::TimerEvent& e )
    {
        double now = ros::Time().now().toSec();
        bool is_nav_data_ok = true;

        // Check imu data
        if(_imu_init) {
            _diagnostic.add("last_imu_data", std::to_string(now - _last_imu_data));
            _diagnostic.add("imu_initialized", "True");
            if(now - _last_imu_data > 1.0) {
                is_nav_data_ok = false;
            }
        }
        else {
             ROS_WARN_STREAM(_name << ": IMU not initialized");
             _diagnostic.add("imu_initialized", "False");
             _diagnostic.add("last_imu_data", std::to_string(0.0));
             is_nav_data_ok = false;
        }

        // Check dvl data
        if(_dvl_init){
            _diagnostic.add("last_dvl_data", std::to_string(now - _last_dvl_data));
            _diagnostic.add("dvl_initialized", "True");
            if(now - _last_dvl_data > 2.0) {
                is_nav_data_ok = false;
            }
        }
        else {
            ROS_WARN_STREAM(_name << ": DVL not initialized");
            _diagnostic.add("dvl_initialized", "False");
            _diagnostic.add("last_dvl_data", std::to_string(0.0));
            is_nav_data_ok = false;
        }

        // Check altitude data
        if(_dvl_init){ // Add an '_altitude_init'? What happens if altitude take from
            _diagnostic.add("last_altitude_data", std::to_string(now - _last_altitude_data));
            if(now - _last_altitude_data > 5.0) {
                is_nav_data_ok = false;
            }
        }
        else {
            _diagnostic.add("last_altitude_data", std::to_string(0.0));
            is_nav_data_ok = false;
        }

        // Check presure data
        if(_pressure_init){
            _diagnostic.add("last_depth_data", std::to_string(now - _last_pressure_data));
            _diagnostic.add("pressure_initialized", "True");
            if( now - _last_pressure_data > 2.0 ) {
                is_nav_data_ok = false;
            }
        }
        else {
            ROS_WARN_STREAM(_name << ": Preassure not initialized");
            _diagnostic.add("pressure_initialized", "False");
            _diagnostic.add("last_depth_data", std::to_string(0.0));
            is_nav_data_ok = false;
        }

        // Check gps data
        if(_use_gps_data){
            if(_gps_init){
                _diagnostic.add("last_gps_data", std::to_string(now - _last_gps_data));
                _diagnostic.add("gps_initialized", "True");
                if(now - _last_gps_data > 3.0) {
                    is_nav_data_ok = false;
                }
            }
            else {
                ROS_WARN_STREAM(_name << ": GPS not initialized");
                _diagnostic.add("gps_initialized", "False");
                _diagnostic.add("last_gps_data", std::to_string(0.0));
                is_nav_data_ok = false;
            }
        }

        // Check current freq
        _diagnostic.add( "freq", std::to_string( _diagnostic.getCurrentFreq() ) );
        if( _diagnostic.getCurrentFreq() < 20 ) {
            is_nav_data_ok = false;
        }

        // If filter or NED not initialized set to Warning
        if( !_ekf_init || !_ned_init ) {
            is_nav_data_ok = false;
            ROS_WARN_STREAM("EKF or NED not yet init");
        }

        // If all nav data is ok set navigator to Ok
        if(is_nav_data_ok && !_ned_error) {
            _diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::OK);
        }
        else {
            _diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::WARN);
        }

        // If 10 times in warning change state to ERROR
        // if( _ekf_init && _ned_init && _diagnostic.getTimesInWarning() >= 10 ) {
        //     _diagnostic.setLevel( diagnostic_msgs::DiagnosticStatus::ERROR );
        // }

        if( !_ekf_init ) ROS_WARN_STREAM( _name << ": EKF not initialized" );
        if( !_ned_init ) ROS_WARN_STREAM( _name << ": NED not initialized" );
        // if( !is_nav_data_ok ) ROS_WARN_STREAM( _name << ": Missing NAV data" );
    }


    bool
    addLandmarkSrv(cola2_msgs::AddLandmark::Request &req,
                   cola2_msgs::AddLandmark::Response &res ) {
        std::cout << "[Navigator] Add landmark\n";
        Eigen::VectorXd landmark = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd landmark_cov = Eigen::MatrixXd::Zero(6, 6);
        landmark(0) = req.landmark.pose.position.x;
        landmark(1) = req.landmark.pose.position.y;
        landmark(2) = req.landmark.pose.position.z;
        Eigen::Vector3d rpy = getRPY(
            Eigen::Quaterniond(req.landmark.pose.orientation.w,
                               req.landmark.pose.orientation.x,
                               req.landmark.pose.orientation.y,
                               req.landmark.pose.orientation.z).toRotationMatrix());
        landmark.tail(3) = rpy;
        for (unsigned int i = 0; i < 6; i++ ) {
            for (unsigned int j = 0; j < 6; j++) {
                landmark_cov(i, j) = req.landmark.covariance.at(i*6+j);
            }
        }
        _ekf_slam_auv->addLandmark(landmark, landmark_cov, req.id.data);
        res.success = true;
        return true;
    }


    bool
    resetNavigation(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res) {
        std::cout << "[Navigator] Reset navigation...\n";
        getConfig();
        initFilter();
        return true;
    }

    bool
    resetLandmarks(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res) {
        std::cout << "[Navigator] Reset landmarks...\n";
        _ekf_slam_auv->resetLandmarks();
        return true;
    }

    bool
    resetNavigationAction(cola2_msgs::Action::Request &req,
                          cola2_msgs::Action::Response &res) {
        std::cout << "[Navigator] Reset navigation through action...\n";
        getConfig();
        initFilter();
        return true;
    }

    bool
    setDepthSensorOffset( std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res ) {
        // std::cout << "_surface_to_depth_sensor_distance: " << _surface_to_depth_sensor_distance << "\n";
        // std::cout << "_last_pressure: " << _last_pressure << "\n";

        if(std::abs(_surface_to_depth_sensor_distance - _last_pressure) < 2.0) {
          _depth_sensor_offset = _surface_to_depth_sensor_distance - _last_pressure;
          std::cout << "[Navigator] Set depth sensor offset at " << _depth_sensor_offset << ".\n";
        }
        else {
          std::cout << "[Navigator] Error setting depth ofsset! " << _surface_to_depth_sensor_distance - _last_pressure << ".\n";
        }
        return true;
    }

    void
    initFilter() {
        _ned_init = false;
        _ekf_init = false;
        _imu_init = false;
        _diagnostic.add( "ned_init", "False" );
        _diagnostic.add( "ekf_init", "False" );
        _diagnostic.add( "imu_init", "False" );

        // getConfig();

        // Init NED from config file if necessary
        if( !_initialize_ned_from_gps ) {
            std::cout << "init ned from config file\n";
            _ned = new Ned( _init_latitude, _init_longitude, 0.0 );
            _ned_init = true;
            _diagnostic.add( "ned_init", "True" );
        }

        // Init filter
        _frame_ids.clear();
        _ekf_slam_auv = new EkfSlamAuv( 6, _q_var );
        if( !_initialize_filter_from_gps ) {
            // Initialize filter at position (0, 0)
            std::cout << "init filter to ( 0, 0 )\n";
            Eigen::VectorXd x = Eigen::MatrixXd::Zero(6, 1);
            _ekf_slam_auv->initEkf( x, _p_var );
            _ekf_init = true;
            _diagnostic.add( "ekf_init", "True" );
        }
    }


    void
    updateBodyForceReq( const ros::MessageEvent<auv_msgs::BodyForceReq const> & msg )
    {
        // The model is obtained from
        // AXIS | Velocity (m/s) | Force
        // ---------------------------------
        //   X  | -0.1 m/s       |  ~ -18N
        //      | 0.1 m/s        |  ~ 17N
        //      | -0.3 m/s       |  ~ -55N
        //      | 0.3 m/s        |  ~ 60N
        // ---------------------------------
        //   Y  | -0.2 m/s       |  ~ -55N
        //      | 0.2 m/s        |  ~ 60N
        // ---------------------------------
        //   Z  | -0.25 m/s      |  ~ -25N
        //      | 0.25 m/s       |  ~ 110N
        // ---------------------------------

        //TODO: Warning! This transformation is hard coded!
        Eigen::Quaterniond rot = euler2Quaternion( deg2Rad( 180.0 ), deg2Rad( 0.0 ), deg2Rad( -45.0 ) );

        //TODO: Warning! The coriolis velocity is set to zero!
        Eigen::Vector3d trans(0.0, 0.0, 0.0); //!!!
        Eigen::Vector3d rate(0.0, 0.0, 0.0); //!!!

        double u = 0.0;
        double v = 0.0;
        double w = 0.0;

        if( not msg.getMessage()->disable_axis.x ) {
            u = msg.getMessage()->wrench.force.x / 200.0;
        }

        if( not msg.getMessage()->disable_axis.y ) {
            v = msg.getMessage()->wrench.force.y / 275.0;
        }

        if( not msg.getMessage()->disable_axis.z ) {
            if ( msg.getMessage()->wrench.force.z > 0.0 ) {
                w = msg.getMessage()->wrench.force.z / 440.0;
            }
            else {
                w = msg.getMessage()->wrench.force.z / 100.0;
            }
        }
        _model_velocity << u, v, w;
        //std::cout << "Model velocity: \n" << _model_velocity << "\n";
        _model_velocity = transformations::linearVelocity( _model_velocity, rate, rot, trans );
        //std::cout << "Model velocity rotated: \n" << _model_velocity << "\n";
    }

    void
    updateImu( const ros::MessageEvent<sensor_msgs::Imu const> & msg )
    {
        _last_imu_data = msg.getMessage()->header.stamp.toSec();

        if ( _ekf_init && _ned_init ) {
            // Check sensor frequency
            _diagnostic.check_frequency();

            // Check if sensor frame_id is defined
            checkFrameId( msg.getMessage()->header.frame_id );

            // std::cout << "Received IMU data\n";
            // TODO: Because IMU data must be in the vehicle frame, rotate orientation if necessary
            Eigen::Quaterniond orientation( msg.getMessage()->orientation.w,
                                            msg.getMessage()->orientation.x,
                                            msg.getMessage()->orientation.y,
                                            msg.getMessage()->orientation.z );

            //std::cout << "orientation: " << orientation.w() << ", " << orientation.x() << ", " << orientation.y() << ", " << orientation.z() << "\n";

            // We supose that the imu computes angular velocity and its covariance.
            Eigen::Vector3d angular_velocity( msg.getMessage()->angular_velocity.x,
                                              msg.getMessage()->angular_velocity.y,
                                              msg.getMessage()->angular_velocity.z );

            Eigen::MatrixXd orientation_cov = Eigen::MatrixXd::Zero(3, 3);
            Eigen::MatrixXd angular_velocity_cov = Eigen::MatrixXd::Zero(3, 3);

            for( unsigned int i = 0; i < 3; i++) {
                for( unsigned int j = 0; j < 3; j++) {
                    orientation_cov(i,j) = msg.getMessage()->orientation_covariance.at(i*3 + j);
                    angular_velocity_cov(i,j) = msg.getMessage()->angular_velocity_covariance.at(i*3 + j);
                }
            }

            _ekf_slam_auv->setImuInput( msg.getMessage()->header.frame_id,
                                        msg.getMessage()->header.stamp.toSec(),
                                        orientation,
                                        orientation_cov,
                                        angular_velocity,
                                        angular_velocity_cov,
                                        _declination );

            publish( msg.getMessage()->header.stamp );
            _imu_init = true;
            _diagnostic.add( "imu_init", "True" );
        }
    }


    void
    updateExplorerDvl( const ros::MessageEvent<cola2_msgs::TeledyneExplorerDvl const> & msg )
    {
        _last_dvl_data = msg.getMessage()->header.stamp.toSec();

        if ( _ekf_init && _ned_init ) {
            _diagnostic.check_frequency();

            // Check if sensor frame_id is defined
            checkFrameId( msg.getMessage()->header.frame_id );

            // If dvl_update == 0 --> No update
            // If dvl_update == 1 --> Update wrt bottom
            // If dvl_update == 2 --> Update wrt water
            // If dvl_update == 3 --> Update wrt Model (only if USE_MODEL = true)

            int dvl_update = 0;

            // Take velocity ( bottom if possible )
            Eigen::Vector3d velocity(0.0, 0.0, 0.0);
            if( msg.getMessage()->wi_status == "A" && msg.getMessage()->wi_error > -32 ){
                if( fabs(msg.getMessage()->wi_x_axis) < _dvl_max_v &&
                     fabs(msg.getMessage()->wi_y_axis) < _dvl_max_v &&
                     fabs(msg.getMessage()->wi_z_axis) < _dvl_max_v / 4.0 ) {
                     velocity << msg.getMessage()->wi_x_axis, msg.getMessage()->wi_y_axis, msg.getMessage()->wi_z_axis;
                     dvl_update = 2;
                     _diagnostic.add( "DVL_source", "Water" );
                 }
            }
            if( msg.getMessage()->bi_status == "A" && msg.getMessage()->bi_error > -32 ){
                _bottom_status++;
                if( fabs(msg.getMessage()->bi_x_axis) < _dvl_max_v &&
                    fabs(msg.getMessage()->bi_y_axis) < _dvl_max_v &&
                    fabs(msg.getMessage()->bi_z_axis) < _dvl_max_v / 4.0) {
                    velocity << msg.getMessage()->bi_x_axis, msg.getMessage()->bi_y_axis, msg.getMessage()->bi_z_axis;
                    dvl_update = 1;
                    _diagnostic.add( "DVL_source", "Bottom" );
                }
            }
            else {
                _bottom_status = 0;
            }

            if( USE_MODEL && dvl_update == 0 ) {
                // No DVL data is available but USE_MODEL is true
                dvl_update = 3;
                velocity = _model_velocity;
                _diagnostic.add( "DVL_source", "Model" );
            }

            // Take altitude
            if( _bottom_status > 4 ) {
                _altitude = msg.getMessage()->bd_range;
                std::ostringstream strs;
                strs << _altitude;

                _diagnostic.add( "Altitude", strs.str() );
                _diagnostic.add( "Altitude from echosounder", "false" );
                _diagnostic.add( "Altitude from multibeam", "false" );

                _last_altitude_data = _last_dvl_data;
            }
            else {
                //Check if there is altitude data from an echosounder
                if( _echosounder_range > 0 && ( ros::Time::now().toSec() - _last_echosounder_data) < 2.0) {
                    _altitude = _echosounder_range;
                    std::ostringstream strs;
                    strs << _altitude;
                    _diagnostic.add( "Altitude", strs.str() );
                    _diagnostic.add( "Altitude from echosounder", "true" );
                    _last_altitude_data = _last_echosounder_data;
                }
                else if (_mb_range > 0 && ( ros::Time::now().toSec() - _last_mb_data) < 2.0) {
                    _altitude = _mb_range;
                    std::ostringstream strs;
                    strs << _altitude;
                    _diagnostic.add( "Altitude", strs.str() );
                    _diagnostic.add( "Altitude from multibeam", "true" );
                    _last_altitude_data = _last_mb_data;
                }
                else
                {
                    _altitude = INVALID_ALTITUDE;
                    _diagnostic.add("Altitude", "Invalid altitude");
                    _diagnostic.add( "Altitude from echosounder", "false" );
                    _diagnostic.add( "Altitude from multibeam", "false" );
                }
            }
            // Compute Covariance matrix and call velocity update
            if( dvl_update != 0 ) {
                Eigen::Matrix3d covariance = Eigen::MatrixXd::Identity(3, 3);
                if( dvl_update == 1 ) { // TODO: MISSING --> depth > 1.5 condition !!!
                    // bottom data and vehicle not in surface
                    // std::cout << "_dvl_bottom_covariance.size(): " << _dvl_bottom_covariance.size() << std::endl;
                    assert( _dvl_bottom_covariance.size() == 3 );
                    covariance(0,0) = _dvl_bottom_covariance.at(0);
                    covariance(1,1) = _dvl_bottom_covariance.at(1);
                    covariance(2,2) = _dvl_bottom_covariance.at(2)*2.0;
                }
                if( dvl_update == 2 ) {
                    // no bottom data or vehicle in the surface
                    assert( _dvl_water_covariance.size() == 3 );
                    covariance(0,0) = _dvl_water_covariance.at(0);
                    covariance(1,1) = _dvl_water_covariance.at(1);
                    covariance(2,2) = _dvl_water_covariance.at(2)*2.0;
                }
                if( dvl_update == 3 ) {
                    // no DVL data using the model
                    assert( _dvl_water_covariance.size() == 3 );
                    covariance(0,0) = _dvl_water_covariance.at(0) * 2.0;
                    covariance(1,1) = _dvl_water_covariance.at(1) * 2.0;
                    covariance(2,2) = _dvl_water_covariance.at(2) * 2.0;
                }

                _ekf_slam_auv->velocityUpdate( msg.getMessage()->header.frame_id,
                                               msg.getMessage()->header.stamp.toSec(),
                                               velocity,
                                               covariance );
                if (!_imu_init) std::cout << "Navigator warning: publishing without initialized IMU\n";
                publish( msg.getMessage()->header.stamp );
                _dvl_init = true;
            }
            else {
                _diagnostic.add( "DVL_source", "no_data" );
            }
        }
        //_ekf_slam_auv->showStateVector();
    }


    void
    updatePressure( const ros::MessageEvent<cola2_msgs::PressureSensor const> & msg )
    {
        if (msg.getMessage()->pressure == -99 || msg.getMessage()->temperature == -99) {
            // Last pressure data and time are not updated
            ROS_ERROR_STREAM(_name << ": invalid pressure!\n");
        }
        else {
            _last_pressure_data = msg.getMessage()->header.stamp.toSec();
            _last_pressure = msg.getMessage()->pressure * _bar_to_meters_depth;

          if (_ekf_init && _ned_init) {
              _diagnostic.check_frequency();

              // Check if sensor frame_id is defined
              checkFrameId(msg.getMessage()->header.frame_id);

              Eigen::Vector3d position(0.0, 0.0, _last_pressure + _depth_sensor_offset);
              Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero(3, 3);
              covariance(0, 0) = 9999.0;
              covariance(1, 1) = 9999.0;
              covariance(2, 2) = _pressure_covariance;

              // If vehicle submerged reset GPS counter.
              if (_last_pressure + _depth_sensor_offset > 1.0) {
                  _gps_samples = 0;
              }

              _ekf_slam_auv->positionUpdate( msg.getMessage()->header.frame_id,
                                             msg.getMessage()->header.stamp.toSec(),
                                             position,
                                             covariance );
              if (!_imu_init) std::cout << "Navigator warning: publishing without initialized IMU\n";
              publish( msg.getMessage()->header.stamp );
              _pressure_init = true;
          }
        }
    }


    void
    updateEchosounder( const ros::MessageEvent<sensor_msgs::Range const> & msg )
    {
        if( msg.getMessage()->range > msg.getMessage()->min_range &&
            msg.getMessage()->range < msg.getMessage()->max_range) {
            _echosounder_range = msg.getMessage()->range;
            _last_echosounder_data = msg.getMessage()->header.stamp.toSec();
            //std::cout << "[navigator_s2] New range measurement: " << _echosounder_range << "\n";
        }
        else {
            _echosounder_range = -1.0 ;
        }
    }

    void
    updateMb( const sensor_msgs::Range::Ptr & msg){
        if( msg->range > msg->min_range &&
            msg->range < msg->max_range) {
            _mb_range = msg->range;
            _last_mb_data = msg->header.stamp.toSec();
        }
        else {
            _mb_range = -1.0 ;
        }
    }

    void
    updateGps( const ros::MessageEvent<cola2_msgs::FastraxIt500Gps const> & msg )
    {
        _last_gps_data = msg.getMessage()->header.stamp.toSec();
        // std::cout << "Receive GPS data\n";

        // Check data quality
        if( msg.getMessage()->data_quality >= 1 &&
            msg.getMessage()->latitude_hemisphere >= 0 &&
            msg.getMessage()->longitude_hemisphere >= 0 &&
            msg.getMessage()->h_dop > 0.0 &&
            msg.getMessage()->h_dop < 2.0 ) {

            _diagnostic.check_frequency();
            _gps_samples++;
            // std::cout << " -----> GPS SAMPLES: " << _gps_samples << "\n";
            // std::cout << "_initialize_filter_from_gps: " << _initialize_filter_from_gps << "\n";
            // std::cout << "_ekf_init: " << _ekf_init << "\n";

            // Check if sensor frame_id is defined
            checkFrameId( msg.getMessage()->header.frame_id );

            // Initialize NED and then the filter if necessary
            if( _initialize_ned_from_gps && !_ned_init ) {
                if( _gps_samples >= _gps_samples_to_init ) {
                    // Initialize NED
                    _init_latitude = dms2DegInt( msg.getMessage()->latitude, int(msg.getMessage()->latitude_hemisphere) );
                    _init_longitude = dms2DegInt( msg.getMessage()->longitude, int(msg.getMessage()->longitude_hemisphere) );
                    std:: cout << "Init NED at GPS position: " << _init_latitude << ", " << _init_longitude << "\n";

                    _ned = new Ned( _init_latitude, _init_longitude, 0.0 );
                    std::cout << "NED initialized\n";
                    _ned_init = true;
                    _diagnostic.add( "ned_init", "True" );

                    // Initialize filter at current position
                    // It doesn't matter if _initialize_filter_from_fps is true or not. Happens the same.
                    Eigen::VectorXd x = Eigen::MatrixXd::Zero(6, 1);
                    std::cout << "Init EKF at (0, 0, 0) \n";
                    _ekf_slam_auv->initEkf( x, _p_var );
                    _ekf_slam_auv->setLastPredictionTime( msg.getMessage()->header.stamp.toSec() );
                    std::cout << "Ekf initialized\n";
                    _ekf_init = true;
                    _diagnostic.add( "ekf_init", "True" );

                    // If initialise depth_sensor_offset is on
                    if( _initialize_depth_sensor_offset ) {
                        std_srvs::Empty::Request req;
                        std_srvs::Empty::Response res;
                        setDepthSensorOffset( req, res );
                    }
                }
            } // initialize the filter if necessary
            else if( _initialize_filter_from_gps && !_ekf_init ) {
                if( _gps_samples >= _gps_samples_to_init ) {
                    // transform lat log to NED and init the filter
                    double latitude = dms2DegInt( msg.getMessage()->latitude, int(msg.getMessage()->latitude_hemisphere) );
                    double longitude = dms2DegInt( msg.getMessage()->longitude, int(msg.getMessage()->longitude_hemisphere) );
                    double north, east, depth;
                    _ned->geodetic2Ned( latitude, longitude, 0.0, north, east, depth );
                    std:: cout << "Init filter at position: " << north << ", " << east << "\n";

                    Eigen::VectorXd x = Eigen::MatrixXd::Zero(6, 1);
                    x(0) = north;
                    x(1) = east;
                    _ekf_slam_auv->initEkf( x, _p_var );
                    _ekf_slam_auv->setLastPredictionTime( msg.getMessage()->header.stamp.toSec() );
                    _ekf_init = true;
                    _diagnostic.add( "ekf_init", "True" );

                    // If initialise depth_sensor_offset is on
                    if( _initialize_depth_sensor_offset ) {
                        std_srvs::Empty::Request req;
                        std_srvs::Empty::Response res;
                        setDepthSensorOffset( req, res );
                    }
                }
            }
            else {
                // Take current lat, long and convert it to NED
                double latitude = dms2DegInt( msg.getMessage()->latitude, int(msg.getMessage()->latitude_hemisphere) );
                double longitude = dms2DegInt( msg.getMessage()->longitude, int(msg.getMessage()->longitude_hemisphere) );
                double north, east, depth;
                _ned->geodetic2Ned( latitude, longitude, 0.0, north, east, depth );

                // Publish Current GPS as PoseStamped
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = "world";
                pose.pose.position.x = north;
                pose.pose.position.y = east;
                pose.pose.position.z = depth;
                pose.pose.orientation.x = 0.0;  // pointing upwards
                pose.pose.orientation.y = 0.70710678;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 0.70710678;
                _pub_gps_ned.publish( pose );

                // If everything is correctly initialized update filter with GPS data
                if( _use_gps_data  && _gps_samples >= _gps_samples_to_init ) {
                    // Compute position vector and covariance matrix
                    Eigen::Vector3d position( north, east, 0.0 );
                    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero(3, 3);
                    covariance(0, 0) = _gps_covariance.at(0);
                    covariance(1, 1) = _gps_covariance.at(1);
                    covariance(2, 2) = 9999.0;

                    // Apply update
                    _ekf_slam_auv->positionUpdate( msg.getMessage()->header.frame_id,
                                                   msg.getMessage()->header.stamp.toSec(),
                                                   position,
                                                   covariance );
                    if (!_imu_init) std::cout << "Navigator warning: publishing without initialized IMU\n";
                    publish( msg.getMessage()->header.stamp );
                    _gps_init = true;
                }
            }
        }
        else {
            if (!_ekf_init && _initialize_filter_from_gps){
                _wrong_gps_samples_before_init++;
                ROS_WARN_STREAM("Wrong GPS data before NED initialized\n");
                if (_wrong_gps_samples_before_init > 60) {
                    // If after 100 samples the ned has not been initialized
                    // change GPS configuration to
                    _initialize_ned_from_gps = false;
                    _initialize_filter_from_gps = false;
                    _use_gps_data = false;
                    _gps_init = true;
                    initFilter();
                    _ned_error = true;
                    _diagnostic.add("ned_init", "ERROR");
                    _diagnostic.setLevel(diagnostic_msgs::DiagnosticStatus::WARN);
                    ROS_WARN_STREAM("Impossible to initialize filter with GPS!\n");
                }
            }
        }
        // std::cout << "_ekf_init: " << _ekf_init << ", _initialize_filter_from_gps: " << _initialize_filter_from_gps << "\n";
    }

    void
    updateWorldFixUpdate(const ros::MessageEvent<geometry_msgs::PoseWithCovarianceStamped const> & msg)
    {
        _diagnostic.check_frequency();

        // If everything is correctly initialized update filter with USBL data
        if( _ekf_init && _ned_init ) {
          // Compute position vector and covariance matrix
          Eigen::Vector3d position(msg.getMessage()->pose.pose.position.x,
                                   msg.getMessage()->pose.pose.position.y,
                                   0.0 );

          Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero(3, 3);
          covariance(0, 0) = msg.getMessage()->pose.covariance.at(0);
          covariance(1, 1) = msg.getMessage()->pose.covariance.at(7);
          covariance(2, 2) = 9999.0;

          std::cout << " ----> WORLD FIX UPDATE <----\n";
          std::cout << "position: " << position << "\n";
          std::cout << "covariance: " << covariance << "\n";

          _ekf_slam_auv->positionUpdate(_vehicle_frame_id,
                                        msg.getMessage()->header.stamp.toSec(),
                                        position,
                                        covariance);

          if (!_imu_init) std::cout << "Navigator warning: publishing without initialized IMU\n";
          publish(msg.getMessage()->header.stamp);
        }
    }

    void
    updateUSBLEvologics( const ros::MessageEvent<geometry_msgs::PoseWithCovarianceStamped const> & msg )
    {
        if ( _use_usbl_data ) {
            _diagnostic.check_frequency();

            // Check if sensor frame_id is defined
            checkFrameId( "/evologics" ); //msg.getMessage()->header.frame_id ); --> Should be "world"

            // If everything is correctly initialized update filter with USBL data
            if( _ekf_init && _ned_init ) {
                Eigen::Vector3d position_increment = getPositionIncrementFrom( msg.getMessage()->header.stamp.toSec() );

                if ( position_increment[0] >= 0.0 ) {

                    // Transform USBL position (lat, lon) to NED (north, east)
                    double north, east, depth;
                    _ned->geodetic2Ned( msg.getMessage()->pose.pose.position.x, msg.getMessage()->pose.pose.position.y, 0.0, north, east, depth );

                    std::cout << "USBL data at " << msg.getMessage()->header.stamp.sec << ":\n" << msg.getMessage()->pose.pose.position.x << ", " << msg.getMessage()->pose.pose.position.y << "\n" ;
                    std::cout << "USBL NED data at " << msg.getMessage()->header.stamp.sec << ":\n" << north << ", " << east << "\n" ;
                    std::cout << "Increment from: \n" << position_increment << "\n";

                    geometry_msgs::PointStamped point;
                    point.header.stamp = msg.getMessage()->header.stamp;
                    point.header.frame_id = "world";
                    point.point.x = north;
                    point.point.y = east;
                    point.point.z = _ekf_slam_auv->getStateVector()(2);  // current navigation z
                    _pub_usbl_ned.publish( point );

                    // Compute position vector and covariance matrix
                    Eigen::Vector3d position( north + position_increment[1],
                                              east + position_increment[2],
                                              0.0 );

                    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero(3, 3);
                    covariance(0, 0) = _usbl_covariance.at(0); //msg.getMessage()->pose.covariance.at(0);
                    covariance(1, 1) = _usbl_covariance.at(1); //msg.getMessage()->pose.covariance.at(8);
                    covariance(2, 2) = 9999.0;

                    // TODO: Check Frame ID!!
                    std::cout << " --------------------- \n ";
                    std::cout << "time: " << msg.getMessage()->header.stamp.toSec() << " + " << position_increment[0] << "\n";
                    std::cout << "position: " << position << "\n";
                    std::cout << "covariance: " << covariance << "\n";


                    _ekf_slam_auv->positionUpdate( "/evologics",
                                                   msg.getMessage()->header.stamp.toSec() + position_increment[0],
                                                   position,
                                                   covariance );

                    if (!_imu_init) std::cout << "Navigator warning: publishing without initialized IMU\n";
                    publish( msg.getMessage()->header.stamp + ros::Duration( position_increment[0] ) );
                }
            }
        }
    }


    void
    updateLandmark(const ros::MessageEvent<cola2_msgs::Detection const> & msg)
    {
        // std::cout << "Received Landmark update:\n";
        // std::cout << msg.getPublisherName() << "\n";

        if (_ekf_init && _ned_init &&  msg.getMessage()->detected) {
            _diagnostic.check_frequency();

            // Check if sensor frame_id is defined
            checkFrameId( msg.getMessage()->header.frame_id );

            Eigen::Vector3d position( msg.getMessage()->pose.pose.position.x,
                                      msg.getMessage()->pose.pose.position.y,
                                      msg.getMessage()->pose.pose.position.z );
            // std::cout << "position:\n" << position << "\n";

            Eigen::Quaterniond orientation( msg.getMessage()->pose.pose.orientation.w,
                                            msg.getMessage()->pose.pose.orientation.x,
                                            msg.getMessage()->pose.pose.orientation.y,
                                            msg.getMessage()->pose.pose.orientation.z );

            Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(6, 6);
            covariance(0, 0) = msg.getMessage()->pose.covariance.at(0);
            covariance(1, 1) = msg.getMessage()->pose.covariance.at(7);
            covariance(2, 2) = msg.getMessage()->pose.covariance.at(14);
            covariance(3, 3) = msg.getMessage()->pose.covariance.at(21);
            covariance(4, 4) = msg.getMessage()->pose.covariance.at(28);
            covariance(5, 5) = msg.getMessage()->pose.covariance.at(35);
            // std::cout << "covariance:\n" << covariance << "\n";

            int res = _ekf_slam_auv->landmarkUpdate( msg.getMessage()->header.frame_id,
                                                     msg.getMessage()->id,
                                                     msg.getMessage()->header.stamp.toSec(),
                                                     position,
                                                     orientation,
                                                     covariance );
            if (!_imu_init) std::cout << "Navigator warning: publishing without initialized IMU\n";
            publish( msg.getMessage()->header.stamp );
            ROS_DEBUG_COND(res == 0, "landmark '%s' validated", msg.getMessage()->id.c_str());
            ROS_DEBUG_COND(res == 1, "landmark '%s' not validated yet", msg.getMessage()->id.c_str());
            ROS_DEBUG_COND(res == 2, "landmark '%s' first sight", msg.getMessage()->id.c_str());
            ROS_DEBUG_COND(res == 3, "landmark '%s' updated", msg.getMessage()->id.c_str());
            ROS_DEBUG_COND(res == 4, "landmark '%s' not updated / too big mahalanobis", msg.getMessage()->id.c_str());
            ROS_DEBUG_COND(res == 5, "landmark '%s' no imu / no prediction", msg.getMessage()->id.c_str());
            // _ekf_slam_auv->showStateVector();
        }
    }

    void
    updateRange(const ros::MessageEvent<cola2_msgs::RangeDetection const> & msg)
    {
        // std::cout << "Received Landmark update:\n";
        // std::cout << msg.getPublisherName() << "\n";

        if (_ekf_init && _ned_init) {
            _diagnostic.check_frequency();


            Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(1, 1);
            covariance(0, 0) = msg.getMessage()->sigma;
            // std::cout << "covariance:\n" << covariance << "\n";

            int res = _ekf_slam_auv->rangeUpdate(msg.getMessage()->id,
                                                 msg.getMessage()->header.stamp.toSec(),
                                                 msg.getMessage()->range,
                                                 covariance);

            rangeMarker(msg.getMessage()->id,
                        msg.getMessage()->range,
                        msg.getMessage()->sigma);

            if (!_imu_init) std::cout << "Navigator warning: publishing without initialized IMU\n";
            publish(msg.getMessage()->header.stamp);
            ROS_DEBUG_COND(res == 0, "landmark '%s' validated", msg.getMessage()->id.c_str());
            ROS_DEBUG_COND(res == 1, "landmark '%s' not validated yet", msg.getMessage()->id.c_str());
            ROS_DEBUG_COND(res == 3, "landmark '%s' updated", msg.getMessage()->id.c_str());
            ROS_DEBUG_COND(res == 4, "landmark '%s' not updated / too big mahalanobis", msg.getMessage()->id.c_str());
            ROS_DEBUG_COND(res == 5, "landmark '%s' no imu / no prediction", msg.getMessage()->id.c_str());
            // _ekf_slam_auv->showStateVector();
        }
    }


private:
    // EKF object
    Eigen::VectorXd _p_var;
    Eigen::VectorXd _q_var;

    // Subscribers
    ros::Subscriber _sub_model;
    ros::Subscriber _sub_explorer_dvl;
    ros::Subscriber _sub_pressure;
    ros::Subscriber _sub_echosounder;
    ros::Subscriber _sub_imu;
    ros::Subscriber _sub_gps;
    ros::Subscriber _sub_usbl;
    ros::Subscriber _sub_mb;
    ros::Subscriber _sub_detection;
    ros::Subscriber _sub_range;
    ros::Subscriber _sub_world_fix;

    // Initialization flags
    bool _dvl_init;
    bool _pressure_init;
    bool _imu_init;
    bool _gps_init;
    bool _ekf_init;
    bool _ned_init;
    bool _ned_error;

    // Diagnostics data
    double _last_imu_data;
    double _last_dvl_data;
    double _last_pressure_data;
    double _last_gps_data;
    double _last_altitude_data;
    double _last_echosounder_data;
    double _last_mb_data;

    // Other data
    double _sound_velocity;
    unsigned int _bottom_status;
    unsigned int _gps_samples;
    unsigned int _wrong_gps_samples_before_init;

    double _echosounder_range;
    double _mb_range;

    ros::ServiceServer _reset_navigation_srv;
    ros::ServiceServer _reset_nav_action_srv;
    ros::ServiceServer _reset_landmarks_srv;
    ros::ServiceServer _set_depth_sensor_offset_srv;
    ros::ServiceServer _add_landmark_srv;

    // To be loaded from config file
    Eigen::Quaterniond _auv_imu_rotation;
    double _dvl_max_v;
    double _pressure_covariance;
    std::vector<double> _dvl_bottom_covariance;
    std::vector<double> _dvl_water_covariance;
    std::vector<double> _gps_covariance;
    std::vector<double>  _model_covariance;
    std::vector<double>  _usbl_covariance;
    Eigen::Vector3d _model_velocity;
    double _water_density;
    double _bar_to_meters_depth;

    bool _initialize_ned_from_gps;
    bool _initialize_filter_from_gps;
    bool _use_gps_data;
    bool _use_usbl_data;
    unsigned int _gps_samples_to_init;
    double _declination;
    double _surface_to_depth_sensor_distance;
    double _depth_sensor_offset;
    double _last_pressure;
    bool _initialize_depth_sensor_offset;

    std::vector< std::string > _landmark_topics;

    std::map< std::string, std::vector< double > > _tf_data;

    void
    getConfig(){
        double declination_in_degrees;

        // Load params from ROS param server
        cola2::rosutil::loadParam( "navigator/dvl_max_v", _dvl_max_v);
        cola2::rosutil::loadParam( "navigator/initialize_ned_from_gps", _initialize_ned_from_gps);
        cola2::rosutil::loadParam( "navigator/initialize_filter_from_gps", _initialize_filter_from_gps);
        cola2::rosutil::loadParam( "navigator/use_gps_data", _use_gps_data );
        cola2::rosutil::loadParam( "navigator/ned_latitude", _init_latitude );
        cola2::rosutil::loadParam( "navigator/ned_longitude", _init_longitude );
        cola2::rosutil::loadParam( "navigator/use_usbl_data", _use_usbl_data );
        cola2::rosutil::loadParam( "navigator/declination_in_degrees", declination_in_degrees );
        _declination = deg2Rad( declination_in_degrees );
        cola2::rosutil::loadParam( "navigator/surface_to_depth_sensor_distance", _surface_to_depth_sensor_distance );
        cola2::rosutil::loadParam( "navigator/initialize_depth_sensor_offset", _initialize_depth_sensor_offset );
        cola2::rosutil::loadParam( "navigator/depth_sensor_offset", _depth_sensor_offset );
        cola2::rosutil::loadParam( "navigator/robot_frame_id", _vehicle_frame_id );
        cola2::rosutil::loadParam( "navigator/world_frame_id", _world_frame_id );
        cola2::rosutil::loadParam( "navigator/dvl_bottom_covariance", _dvl_bottom_covariance);
        cola2::rosutil::loadParam( "navigator/dvl_water_covariance", _dvl_water_covariance);
        cola2::rosutil::loadParam( "navigator/gps_covariance", _gps_covariance);
        cola2::rosutil::loadParam( "navigator/pressure_covariance", _pressure_covariance);
        cola2::rosutil::loadParam( "navigator/model_covariance", _model_covariance);
        cola2::rosutil::loadParam( "navigator/usbl_covariance", _usbl_covariance);
        cola2::rosutil::loadParam( "navigator/water_density", _water_density);
    }
};



int
main( int argc, char **argv )
{
    std::cout << "Init G500 navigator\n";

    // Init ROS controller
    ros::init(argc, argv, "navigator");

    // Init Ekf Slam filter
    Eigen::VectorXd q_var = Eigen::MatrixXd::Zero(3, 1);
    q_var << 0.02, 0.02, 0.02;

    Eigen::VectorXd p_var = Eigen::MatrixXd::Zero(6, 1);
    p_var << 0.01, 0.01, 0.5, 0.02, 0.02, 0.02;

    S2RosNavigator s2_navigator( ros::this_node::getName(), p_var );

    // Spin until architecture stops
    ros::spin();

    return 0;
}
