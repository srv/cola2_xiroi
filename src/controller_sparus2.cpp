#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <cola2_lib/cola2_control/IAUVController.h>
#include <cola2_lib/cola2_control/IAUVROSController.h>
#include "cola2_lib/cola2_rosutils/RosUtil.h"
#include "include/SPARUS2Controller.hpp"
#include <cola2_s2/controller_sparus2Config.h>

class SPARUS2ROSController: public IAUVROSController
{
public:

    SPARUS2ROSController( const std::string name,
                       const std::string frame_id ):
        IAUVROSController( name, frame_id )
    { }

    void
    init( SPARUS2Controller * auv_controller_ptr, double period )
    {
        initBase( auv_controller_ptr, period );

        // Init pointer to AUV controller
        _auv_controller = auv_controller_ptr;

        // Init dynamic reconfigure
        _f = boost::bind( &SPARUS2ROSController::setParams, this, _1, _2 );
        _param_server.setCallback( _f );
    }


    void
    setParams( cola2_s2::controller_sparus2Config &config,
               uint32_t level )
    {
        std::vector< double > tcm;
        cola2::rosutil::loadParam( "/controller/TCM", tcm );

        std::vector< double > fcm;
        cola2::rosutil::loadParam( "/controller/FCM", fcm );


        _auv_controller->_thruster_allocator.setParams( config.max_force_thruster_forward,
                                                        config.max_force_thruster_backward,
                                                        config.thruster_distance_yaw,
                                                        tcm);

        _auv_controller->_fin_allocator.setParams( config.max_fin_force,
                                                   config.fin_distance_surge,
                                                   config.fin_distance_yaw,
                                                   config.max_fin_angle,
                                                   config.min_fin_angle,
                                                   config.force_to_fins_ratio,
                                                   fcm );


        std::vector< std::string > keys = {"kp", "ti", "td", "i_limit", "fff"};

        std::vector< std::map< std::string, double > > d_p_params;
        std::vector< double > dpvalues1 = {config.d_p_kp, config.d_p_ti, config.d_p_td, config.d_p_i_limit, config.d_p_fff};
        _auv_controller->addPIDParamToVector( keys, dpvalues1, d_p_params );


        std::vector< std::map< std::string, double > > f_params;
        std::vector< double > fvalues1 = {config.f_roll_kp, config.f_roll_ti, config.f_roll_td, config.f_roll_i_limit, config.f_roll_fff};
        _auv_controller->addPIDParamToVector( keys, fvalues1, f_params );
        std::vector< double > fvalues2 = {config.f_pitch_kp, config.f_pitch_ti, config.f_pitch_td, config.f_pitch_i_limit, config.f_pitch_fff};
        _auv_controller->addPIDParamToVector( keys, fvalues2, f_params );


        std::vector< std::map< std::string, double > > p_params;
        std::vector< double > values1 = {config.p_surge_kp, config.p_surge_ti, config.p_surge_td, config.p_surge_i_limit, config.p_surge_fff};
        _auv_controller->addPIDParamToVector( keys, values1, p_params );
        std::vector< double > values2 = {config.p_sway_kp, config.p_sway_ti, config.p_sway_td, config.p_sway_i_limit, config.p_sway_fff};
        _auv_controller->addPIDParamToVector( keys, values2, p_params );
        std::vector< double > values3 = {config.p_heave_kp, config.p_heave_ti, config.p_heave_td, config.p_heave_i_limit, config.p_heave_fff};
        _auv_controller->addPIDParamToVector( keys, values3, p_params );
        std::vector< double > values4 = {config.p_roll_kp, config.p_roll_ti, config.p_roll_td, config.p_roll_i_limit, config.p_roll_fff};
        _auv_controller->addPIDParamToVector( keys, values4, p_params );
        std::vector< double > values5 = {config.p_pitch_kp, config.p_pitch_ti, config.p_pitch_td, config.p_pitch_i_limit, config.p_pitch_fff};
        _auv_controller->addPIDParamToVector( keys, values5, p_params );
        std::vector< double > values6 = {config.p_yaw_kp, config.p_yaw_ti, config.p_yaw_td, config.p_yaw_i_limit, config.p_yaw_fff};
        _auv_controller->addPIDParamToVector( keys, values6, p_params );

        std::vector< std::map< std::string, double > > t_params;
        std::vector< double > tvalues1 = {config.t_surge_kp, config.t_surge_ti, config.t_surge_td, config.t_surge_i_limit, config.t_surge_fff};
        _auv_controller->addPIDParamToVector( keys, tvalues1, t_params );
        std::vector< double > tvalues2 = {config.t_sway_kp, config.t_sway_ti, config.t_sway_td, config.t_sway_i_limit, config.t_sway_fff};
        _auv_controller->addPIDParamToVector( keys, tvalues2, t_params );
        std::vector< double > tvalues3 = {config.t_heave_kp, config.t_heave_ti, config.t_heave_td, config.t_heave_i_limit, config.t_heave_fff};
        _auv_controller->addPIDParamToVector( keys, tvalues3, t_params );
        std::vector< double > tvalues4 = {config.t_roll_kp, config.t_roll_ti, config.t_roll_td, config.t_roll_i_limit, config.t_roll_fff};
        _auv_controller->addPIDParamToVector( keys, tvalues4, t_params );
        std::vector< double > tvalues5 = {config.t_pitch_kp, config.t_pitch_ti, config.t_pitch_td, config.t_pitch_i_limit, config.t_pitch_fff};
        _auv_controller->addPIDParamToVector( keys, tvalues5, t_params );
        std::vector< double > tvalues6 = {config.t_yaw_kp, config.t_yaw_ti, config.t_yaw_td, config.t_yaw_i_limit, config.t_yaw_fff};
        _auv_controller->addPIDParamToVector( keys, tvalues6, t_params );

        std::vector< std::map< std::string, double > > poly_params;
        std::vector< double > pvalues1 = { config.poly_surge_A, config.poly_surge_B, config.poly_surge_C };
        _auv_controller->addPolyParamToVector( pvalues1, poly_params );
        std::vector< double > pvalues2 = {config.poly_sway_A, config.poly_sway_B, config.poly_sway_C};
        _auv_controller->addPolyParamToVector( pvalues2, poly_params );
        std::vector< double > pvalues3 = {config.poly_heave_A, config.poly_heave_B, config.poly_heave_C};
        _auv_controller->addPolyParamToVector( pvalues3, poly_params );
        std::vector< double > pvalues4 = {config.poly_roll_A, config.poly_roll_B, config.poly_roll_C};
        _auv_controller->addPolyParamToVector( pvalues4, poly_params );
        std::vector< double > pvalues5 = {config.poly_pitch_A, config.poly_pitch_B, config.poly_pitch_C};
        _auv_controller->addPolyParamToVector( pvalues5, poly_params );
        std::vector< double > pvalues6 = {config.poly_yaw_A, config.poly_yaw_B, config.poly_yaw_C};
        _auv_controller->addPolyParamToVector( pvalues6, poly_params );


        double poly_surge_percentatge = config.poly_surge_percentatge;
        double poly_sway_percentatge = config.poly_sway_percentatge;
        double poly_heave_percentatge = config.poly_heave_percentatge;
        double poly_roll_percentatge = config.poly_roll_percentatge;
        double poly_pitch_percentatge = config.poly_pitch_percentatge;
        double poly_yaw_percentatge = config.poly_yaw_percentatge;
        std::vector< double > poly_percentatge = { poly_surge_percentatge, poly_sway_percentatge, poly_heave_percentatge, poly_roll_percentatge, poly_pitch_percentatge, poly_yaw_percentatge };


        std::vector< double > max_wrench = { config.max_wrench_X, config.max_wrench_Y, config.max_wrench_Z, config.max_wrench_Roll, config.max_wrench_Pitch, config.max_wrench_Yaw };
        std::vector< double > max_velocity = { config.max_velocity_x, config.max_velocity_y, config.max_velocity_z, config.max_velocity_roll, config.max_velocity_pitch, config.max_velocity_yaw };

        _auv_controller->setMaxVelocity( max_velocity );
        _auv_controller->setMaxWrench( max_wrench );

        // Change Params in C++ Class
        _auv_controller->setControllerParams( d_p_params, f_params, p_params, t_params, poly_params, poly_percentatge );

        std::cout << "config.enable_thrusters: " << config.enable_thrusters << std::endl;
        if(config.enable_thrusters) {
          std::cout << "Thruster enabled\n";
          _auv_controller->setThrusterAllocator(true);
        }
        else {
          std::cout << "Thruster disabled\n";
          _auv_controller->setThrusterAllocator(false);
        }

        std::cout << "Parameters changed!" << std::endl;
    }


private:
    // AUV controller ptr.
    SPARUS2Controller *_auv_controller;

    // Dynamic reconfigure parameters
    dynamic_reconfigure::Server< cola2_s2::controller_sparus2Config > _param_server;
    dynamic_reconfigure::Server< cola2_s2::controller_sparus2Config >::CallbackType _f;
};


int
main( int argc, char **argv )
{

    // Init ROS controller
    ros::init(argc, argv, "controller");

    // Define period --> 0.1 = 10Hz
    double period = 0.1;

    // Init SPARUS2 controller
    SPARUS2Controller *auv_ctrl_ptr;
    auv_ctrl_ptr = new SPARUS2Controller( period, 6, 3, 2 );  // period, number of DOF's, number of thrusters, number of fins,

    // Init ROS node
    SPARUS2ROSController sparus2_ros_controller( ros::this_node::getName(), "/sparus2" );

    // Initialize controller pointer into ROS node
    sparus2_ros_controller.init( auv_ctrl_ptr, period );

    // Spin until architecture stops
    ros::spin();

    return 0;
}
