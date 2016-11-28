#ifndef __SPARUS2_CONTROLLER__
#define __SPARUS2_CONTROLLER__

#include <cola2_lib/cola2_control/Merge.h>
#include <cola2_lib/cola2_control/NDofController.h>
#include <cola2_lib/cola2_control/Pid.h>
#include <cola2_lib/cola2_control/Poly.h>
#include <cola2_lib/cola2_control/IAUVController.h>
#include "SPARUS2ThrusterAllocator.hpp"
#include "SPARUS2FinAllocator.hpp"
#include <algorithm>
#include <Eigen/Dense>


class SPARUS2Controller: public IAUVController {

public:

    SPARUS2Controller( double period,
                       unsigned int n_dof,
                       unsigned int n_thrusters,
                       unsigned int n_fins = 0 ):
        IAUVController( period, n_dof, n_thrusters, n_fins ),
        _thruster_allocator( n_thrusters ),
        _fin_allocator( n_fins ),
        _n_thrusters( n_thrusters ),
        _pose_controller( n_dof ),
        _twist_factor( n_dof ),
        _old_twist( n_dof ),
        _twist_controller( n_dof ),
        _force_factor( n_dof ),
        _twist_poly_controller( n_dof ),
        _poly_percentatge( n_dof )
    {
        // Init controllers
        initFinsControllers();
        initPoseController();
        initTwistController();
        initTwistPolyController();
    }


    void
    initFinsControllers()
    {
        _d_to_p_controller = new Pid( "depth_to_pitch" );

        _f_roll = new Pid( "roll_to_force" );
        _f_pitch = new Pid( "pitch_to_force" );
    }

    void
    initPoseController()
    {
        _p_surge = new Pid( "pose_surge" );
        _pose_controller.addController( _p_surge );

        _p_sway = new Pid( "pose_sway" );
        _pose_controller.addController( _p_sway );

        _p_heave = new Pid( "pose_heave" );
        _pose_controller.addController( _p_heave );

        _p_roll = new Pid( "pose_roll" );
        _pose_controller.addController( _p_roll );

        _p_pitch = new Pid( "pose_pitch" );
        _pose_controller.addController( _p_pitch );

        _p_yaw = new Pid( "pose_yaw" );
        _pose_controller.addController( _p_yaw );
    }

    void
    initTwistController()
    {
        _t_surge = new Pid( "twist_surge" );
        _twist_controller.addController( _t_surge );

        _t_sway = new Pid( "twist_sway" );
        _twist_controller.addController( _t_sway );

        _t_heave = new Pid( "twist_heave" );
        _twist_controller.addController( _t_heave );

        _t_roll = new Pid( "twist_roll" );
        _twist_controller.addController( _t_roll );

        _t_pitch = new Pid( "twist_pitch" );
        _twist_controller.addController( _t_pitch );

        _t_yaw = new Pid( "twist_yaw" );
        _twist_controller.addController( _t_yaw );
    }


    void
    initTwistPolyController()
    {
        _tp_surge = new Poly( "twist_poly_surge" );
        _twist_poly_controller.addController( _tp_surge );

        _tp_sway = new Poly( "twist_poly_sway" );
        _twist_poly_controller.addController( _tp_sway );

        _tp_heave = new Poly( "twist_poly_heave" );
        _twist_poly_controller.addController( _tp_heave );

        _tp_roll = new Poly( "twist_poly_roll" );
        _twist_poly_controller.addController( _tp_roll );

        _tp_pitch = new Poly( "twist_poly_pitch" );
        _twist_poly_controller.addController( _tp_pitch );

        _tp_yaw = new Poly( "twist_poly_yaw" );
        _twist_poly_controller.addController( _tp_yaw );
    }

    void
    setControllerParams( const std::vector< std::map< std::string, double > > d_p_params,
                         const std::vector< std::map< std::string, double > > f_params,
                         const std::vector< std::map< std::string, double > > p_params,
                         const std::vector< std::map< std::string, double > > t_params,
                         const std::vector< std::map< std::string, double > > poly_params,
                         std::vector< double > poly_percentatge)
    {
        _d_to_p_controller->setParameters( d_p_params.at(0) );
        _f_roll->setParameters( f_params.at(0) );
        _f_pitch->setParameters( f_params.at(1) );

        _pose_controller.setControllerParams( p_params );
        _twist_controller.setControllerParams( t_params );
        _twist_poly_controller.setControllerParams( poly_params );

        _twist_factor = { 2.0, 0.0, 0.3, 0.0, 0.0, 0.6 };
        _force_factor = { 154.0, 0, 30.0, 3.64, 17.15, 14.43 };
        _old_twist = { 0, 0, 0, 0, 0, 0 };
        _poly_percentatge = poly_percentatge;

        _d_to_p_controller->reset();
        _f_roll->reset();
        _f_pitch->reset();
        _pose_controller.reset();
        _twist_controller.reset();
        _twist_poly_controller.reset();
    }

    void
    reset( )
    {
        _pose_controller.reset();
        _twist_controller.reset();
        _twist_poly_controller.reset();
    }

    void
    iteration( double current_time )
    {
        // Merge pose requests
        Request desired_pose = _pose_merge.merge( current_time );
        _merged_pose = desired_pose;

        float th0_coef = 1;
        float boy_coef = 0.0; //TODO: Use to be 1.0 (narcis)
        bool apply_boy_coef = false;

        // SPARUS II fins
        // ***********************************************************************************************************************************

        // Fins wrench requests
        Request fins_wrench ( desired_pose );

        std::vector< double > pose_values = desired_pose.getValues();
        std::vector< bool > pose_axis_values = desired_pose.getDisabledAxis();

        std::vector< double > fins_values = fins_wrench.getValues();
        std::vector< bool > fins_axis = fins_wrench.getDisabledAxis();

        if ( ( _is_fin_allocator_enable ) and ( _twist_feedback[0] > 0.0 ) ) {

            // Takes values between -pi and pi
            double desired_pitch;

            // If pitch pose axis is disabled -> 'automatic' pitch mode (2 cases: operating with velocity (PITCH MODE) or with 'automatic' pose (DEPTH MODE))
            if ( pose_axis_values[4] ) {

                // When operating the vehicle with velocity and there is not any WWR, do not compute the PID depth to pitch
                if ( pose_axis_values[2] ) {

                    // With velocity control the desired pitch is 0.0
                    desired_pitch = 0.0;

                    // std::cout << "XXXXXXXXXXXXXXXXX HEAVE VELOCITY CONTROL ENABLED XXXXXXXXXXXXXXXXX \n";
                    // std::cout << "XXXXXXXXXXXXXXXXXX SETTING DESIRED PITCH TO 0.0 XXXXXXXXXXXXXXXXXX \n\n";
                }
                // Modify desired depth pose to reach it with pitch
                else {

                    // Membership for depth
                    //float s; //shallow
                    float d; //deep
                    float low_d = 0.2;
                    float high_d = 1.0;

                    // Memberships for surge velocity
                    float s1; // slow
                    float s2; // normal
                    float s3; // fast
                    float low_s = 0.7; //0.1
                    float high_s = 0.9; //0.25
                    float low_f = 1.2; //0.35
                    float high_f = 1.4; //0.5

                    // Compute memership functions
                    d = ( 1 / ( high_d - low_d ) ) * _pose_feedback[2] + ( 1 - ( 1 / ( high_d - low_d ) ) * high_d );
                    if( d > 1 ) d = 1;
                    if( d < 0 ) d = 0;

                    s1 = ( -1 / ( high_s - low_s ) ) * _twist_feedback[0] + ( high_s / ( high_s - low_s ) );
                    if( s1 > 1 ) s1 = 1;
                    if( s1 < 0 ) s1 = 0;

                    s3 = ( 1 / ( high_f - low_f ) ) * _twist_feedback[0] + ( -low_f / ( high_f - low_f ) );
                    if( s3 > 1 ) s3 = 1;
                    if( s3 < 0 ) s3 = 0;

                    if ( _twist_feedback[0] < high_s ) {
                        s2 = ( 1 - s1 );
                    }
                    else if ( _twist_feedback[0] > low_f ) {
                        s2 = ( 1 - s3 );
                    }
                    else {
                        s2 = 1;
                    }

                    // PID depth -> pitch ( taking in account the depth )
                    // Applying an offset to correct the buoyancy
                    float pitch_offset = 0.0;
                    /*if ( ( ( ( pose_values[2] + 1.5 ) > _pose_feedback[2] ) and ( ( pose_values[2] - 1.5 ) < _pose_feedback[2] ) ) or ( pose_values[2] > _pose_feedback[2] ) )  {
                        pitch_offset = - atanf( fabs( _twist_feedback[2] ) / _twist_feedback[0] );
                    }*/
                    desired_pitch = d * (3.14159265359 / 2) * ( -_d_to_p_controller->compute( current_time, pose_values[2], _pose_feedback[2] ) + pitch_offset );

                    // Compute vertical thruster correction coeficients
                    th0_coef = 1 - d + s1 * d;
                    boy_coef = 1 - s3 * d;

                    // std::cout << "XXXXXXXXXXXXXXXXXXXX DEPTH TO PITCH ENABLED XXXXXXXXXXXXXXXXXXXX \n";
                    // std::cout << "XXXXXXXXXXXXXXXXXXXX REACH DEPTH WITH PITCH XXXXXXXXXXXXXXXXXXXX \n\n";

                    // Rules
                    if ( ( d == 0 ) or ( s1 == 1 ) ) {
                        // depth control by th0 at 100%
                        // keeping 'bouyancy' at 100%
                        // desired pitch is 0 degrees
                        // std::cout << "XXXXXXXXXXXXXXXXXX IMPOSSIBLE XXXXXXXXXXXXXXXXXX \n\n";
                        pose_axis_values[2] = false;
                        th0_coef = 1;
                        boy_coef = 1;
                        desired_pitch = 0.0;
                    }
                    else if ( ( s1 > 0 ) and ( s2 > 0 ) and ( s3 == 0 ) ) {
                        // scaling depth control by th0
                        // keeping 'bouyancy' at 100%
                        // scaling desired pitch
                        // std::cout << "XXXXXXXXXXXXXXXXXXXX STATE 2 XXXXXXXXXXXXXXXXXXX \n\n";
                        pose_axis_values[2] = false;
                        desired_pitch = s2 * desired_pitch;
                    }
                    else if ( ( s1 == 0 ) and ( s2 > 0 ) and ( s3 == 0 ) ) {
                        // depth control by th0 at 0%
                        // keeping 'bouyancy' at 100%
                        // desired pitch is the DtoP PID controller result
                        // std::cout << "XXXXXXXXXXXXXXXXXXXX STATE 3 XXXXXXXXXXXXXXXXXXX \n\n";
                        if ( d == 1 ) {
                            pose_axis_values[2] = true;
                        }
                        apply_boy_coef = true;
                    }
                    else if ( ( s1 == 0 ) and ( s2 > 0 ) and ( s3 > 0 ) ) {
                        // depth control by th0 at 0%
                        // scaling 'bouyancy'
                        // desired pitch is the DtoP PID controller result
                        // std::cout << "XXXXXXXXXXXXXXXXXXXX STATE 4 XXXXXXXXXXXXXXXXXXX \n\n";
                        if ( d == 1 ) {
                            pose_axis_values[2] = true;
                        }
                        apply_boy_coef = true;
                    }
                    else if ( ( s1 == 0 ) and ( s2 == 0 ) and ( s3 > 0 ) ) {
                        // depth control by th0 at 0%
                        // 'buoyancy' at 0%
                        // desired pitch is the DtoP PID controller result
                        // std::cout << "XXXXXXXXXXXXXXXXXX JUST FINS XXXXXXXXXXXXXXXXXX \n\n";
                        if ( d == 1 ) {
                            pose_axis_values[2] = true;
                        }
                    }

                    // Saturate desired pitch
                    desired_pitch = _saturateValue( desired_pitch, 0.785 );

                    // std::cout << "---------- V.TH at " << th0_coef << " % \n";
                    // std::cout << "---------- BUO. at " << boy_coef << " % \n\n";
                    // std::cout << "---- PITCH OFFSET: " << pitch_offset * 180 / 3.1415 << "\n";
                    // std::cout << "------ DES. PITCH: " << desired_pitch * 180 / 3.1415 << "\n\n";
                }
            }
            // If pitch pose axis is enabled -> direct pitch mode (1 case: operating with 'manual' pose (PITCH MODE))
            else  {
                // The desired pitch is directly the input pitch
                desired_pitch = pose_values[4];

                // std::cout << "XXXXXXXXXXXXXXXXXXXX DIRECT PITCH XXXXXXXXXXXXXXXXXXXX \n\n";
            }

            // PIDs: ( roll -> force ) and ( pitch -> force )
            fins_values[3] = _saturateValue( _f_roll->compute(current_time, pose_values[3], _pose_feedback[3]) * _force_factor.at(3), _max_wrench.at(3) );
            fins_values[4] = _saturateValue( _f_pitch->compute(current_time, desired_pitch, _pose_feedback[4]) * _force_factor.at(4), _max_wrench.at(4) );
        }
        else {
            // Set roll and pitch desired torque to 0 due to fins are disabled
            fins_values[3] = 0.0;
            fins_values[4] = 0.0;
        }
        // Disable the roll and pitch axis for the twist controller (is not able to control this DOFs)
        pose_axis_values[3] = true;
        pose_axis_values[4] = true;

        // Set the modified axis and values to the pose requester
        desired_pose.setValues(pose_values);
        desired_pose.setDisabledAxis(pose_axis_values);

        // Enable the roll and pitch axis in the fins requester             TODO: take the input values
        fins_axis[3] = false;
        fins_axis[4] = false;

        // Set the computed axis and values to the fins requester
        fins_wrench.setValues(fins_values);
        fins_wrench.setDisabledAxis(fins_axis);

        // std::cout << "Fins controller result: \n" << fins_wrench << std::endl;
        // ***********************************************************************************************************************************



        if( _is_pose_controller_enable ) {

            // Initialize the twist req. that will be generated by the pose controller
            Request velocity_req( desired_pose );

            // Compute pose error
            std::vector< double >  pose_error = computeError( desired_pose.getValues(), _pose_feedback );

            // If desired pose in is near to 0 and current pose is near to 0 disable Z axis
            if( _pose_feedback.at(2) < 1.0 && desired_pose.getValues().at(2) < 1.0 ) {
                std::vector< bool > disable_axis = velocity_req.getDisabledAxis();
                disable_axis.at(2) = true;
                velocity_req.setDisabledAxis(disable_axis);
                // std::cout << "Disable Z axis because to close to surface\n";
            }

            // Compute PID between pose error and zero
            desired_pose.setValues( pose_error );
            std::vector< double > zero(6, 0.0);
            std::vector< double > pose_ctrl_tau = _pose_controller.compute( current_time, desired_pose, zero );

            // Normalize output to max velocity
            assert( pose_ctrl_tau.size() == _max_velocity.size() );
            for( unsigned int i = 0; i < pose_ctrl_tau.size(); i++ ) {
                if ( i == 2 ) {
                    // Scale desired velocity according to fins controller
                    pose_ctrl_tau.at(i) = th0_coef * _saturateValue( pose_ctrl_tau.at(i) * _twist_factor[i], _max_velocity.at(i) );
                }
                else {
                    pose_ctrl_tau.at(i) = _saturateValue( pose_ctrl_tau.at(i) * _twist_factor[i], _max_velocity.at(i) );
                }
            }

            // Add Pose controller response to body velocity requests
            velocity_req.setValues( pose_ctrl_tau );
            // std::cout << "Pose controller result: \n" << velocity_req << std::endl;
            _twist_merge.addRequest( velocity_req );
        }

        // Merge twist request
        Request desired_twist = _twist_merge.merge( current_time );
        _merged_twist = desired_twist;



        if( _is_velocity_controller_enable ) {
            // Initialize the wrench_req req. that will be generated by the twist controller
            Request wrench_req( desired_twist );

            // Apply a non symmetric ramp to the twist input
            std::vector< double > twist_step = { 0.1, 0, 0.0, 0.0, 0.00, 0.02 };
            std::vector< bool > apply_step = {true, false, false, false, false, true};
            std::vector< double > des_twist = desired_twist.getValues();
            for ( unsigned int i = 0; i < 6; i++ ) {
                if ( apply_step[i] ) {
                    if ( ( des_twist[i] > _old_twist[i] + twist_step[i] ) and ( des_twist[i] > twist_step[i] ) ) {
                        if ( ( _old_twist[i] + twist_step[i] ) > twist_step[i] ) {
                            des_twist[i] = _old_twist[i] + twist_step[i] ;
                        }
                        else {
                            des_twist[i] = twist_step[i];
                        }
                    }
                    else if ( ( des_twist[i] < _old_twist[i] - twist_step[i] ) and ( des_twist[i] < -twist_step[i] ) ) {
                        if ( ( _old_twist[i] - twist_step[i]) < -twist_step[i] ) {
                            des_twist[i] = _old_twist[i] - twist_step[i] ;
                        }
                        else {
                            des_twist[i] = -twist_step[i] ;
                        }
                    }
                    _old_twist[i] = des_twist[i];
                    desired_twist.setValues(des_twist);
                }
            }

            // Compute twist control (and set the resulting wrench req.)
            std::vector< double > pid_twist = _twist_controller.compute( current_time,
                                                                         desired_twist,
                                                                         _twist_feedback );

            // For the poly input take in account the currents ( desired_twist = desired_twist - _currents_feedback )

            std::vector< double > poly_twist = _twist_poly_controller.compute( current_time,
                                                                               desired_twist,
                                                                               _twist_feedback);

            // Normalize output to max force
            assert( pid_twist.size() == poly_twist.size() );
            std::vector< double > total;
            for ( unsigned int i = 0; i < pid_twist.size(); i++ ) {
                if ( i == 2 ) {
                    // Add buoyancy
                    pid_twist.at(i) = pid_twist.at(i) + 0.10 * boy_coef;                                                   //TODO: take this value from the .yaml

                    if ( apply_boy_coef ) {
                        std::vector< bool > axis = wrench_req.getDisabledAxis();
                        axis[2] = false;
                        wrench_req.setDisabledAxis( axis );
                    }
                }

                // Scale and saturate force
                total.push_back( _saturateValue( _saturateValue( pid_twist.at(i), 1.0 ) * _force_factor[i] + poly_twist.at(i) * _poly_percentatge.at(i), _max_wrench.at(i) ) );
                //std::cout << i << " Total: " << _saturateValue( scaled[i], _max_wrench.at(i) ) << "\n";
                //std::cout << "  PID:  " <<  pid_twist.at(i) << "\n";
                //std::cout << "  Poly: " <<  poly_twist.at(i) << "\n\n";
            }

            wrench_req.setValues( total );
            // std::cout << "Twist controller result: \n" << wrench_req << std::endl;
            _wrench_merge.addRequest( wrench_req );
        }

        // Merge wrench req. and save them
        _merged_wrench = _wrench_merge.merge( current_time );

        // Compute thruster setpoints -> done in the interatin loop

        // Compute fin setpoints
        //_fin_setpoints = _fin_allocator.calibration( fins_setpoints );
        _fin_setpoints = _fin_allocator.compute( fins_wrench, _thruster_setpoints, _twist_feedback );
    }


    std::vector< double >
    computeError( const std::vector< double > setpoint,
                  const std::vector< double > feedback )
    {
        // WARNING: This function is intended for 6 DoF controllers only!
        assert( setpoint.size() == 6 );
        assert( feedback.size() == 6 );

        std::vector< double > error( 6, 0.0 );
        poseError( setpoint, feedback, error );
        error.at( 2 ) = setpoint.at( 2 ) - feedback.at( 2 );
        error.at( 3 ) = _normalizeAngle( setpoint.at( 3 ) - feedback.at( 3 ) );
        error.at( 4 ) = _normalizeAngle( setpoint.at( 4 ) - feedback.at( 4 ) );
        error.at( 5 ) = _normalizeAngle( setpoint.at( 5 ) - feedback.at( 5 ) );

        return error;
    }


    void
    poseError( const std::vector< double > setpoint,
               const std::vector< double > feedback,
               std::vector< double >& error )
    {
        double yaw = feedback.at( 5 );
        Eigen::MatrixXd m( 3, 3 );
        m( 0, 0 ) = cos( yaw );     m( 0, 1 ) = -sin( yaw );    m( 0, 2 ) = 0.0;
        m( 1, 0 ) = sin( yaw );     m( 1, 1 ) = cos( yaw );     m( 1, 2 ) = 0.0;
        m( 2, 0 ) = 0.0;            m( 2, 1 ) = 0.0;            m( 2, 2 ) = 1.0;
        //std::cout << "m:\n" << m << "\n\n";

        double pitch = feedback.at( 4 );
        Eigen::MatrixXd n( 3, 3 );
        n( 0, 0 ) = cos( pitch );   n( 0, 1 ) = 0.0;            n( 0, 2 ) = sin( pitch );
        n( 1, 0 ) = 0.0;            n( 1, 1 ) = 1.0;            n( 1, 2 ) = 0.0;
        n( 2, 0 ) = -sin( pitch );  n( 2, 1 ) = 0.0;            n( 2, 2 ) = cos( pitch );
        //std::cout << "n:\n" << n << "\n\n";

        double roll = feedback.at( 3 );
        Eigen::MatrixXd o( 3, 3 );
        o( 0, 0 ) = 1.0;            o( 0, 1 ) = 0.0;            o( 0, 2 ) = 0.0;
        o( 1, 0 ) = 0.0;            o( 1, 1 ) = cos( roll );    o( 1, 2 ) = -sin( roll );
        o( 2, 0 ) = 0.0;            o( 2, 1 ) = sin( roll );    o( 2, 2 ) = cos( roll );
        //std::cout << "o:\n" << o << "\n\n";

        Eigen::MatrixXd p( 3, 1 );
        p( 0, 0 ) = feedback.at( 0 );
        p( 1, 0 ) = feedback.at( 1 );
        p( 2, 0 ) = 0.0;
        //std::cout << "p:\n" << p << "\n\n";

        // Rotation: yaw -> pitch -> roll
        Eigen::MatrixXd rotZYX = m * n * o;
        //std::cout << "rotation:\n" << rotZYX << "\n\n";

        Eigen::MatrixXd aux = -rotZYX.transpose() * p;
        //std::cout << "aux:\n" << aux << "\n\n";

        Eigen::MatrixXd T( 4, 4 );
        T( 0, 0 ) = rotZYX.transpose()( 0, 0 );  T( 0, 1 ) = rotZYX.transpose()( 0, 1 );  T( 0, 2 ) = rotZYX.transpose()( 0, 2 );  T( 0, 3 ) = aux( 0, 0 );
        T( 1, 0 ) = rotZYX.transpose()( 1, 0 );  T( 1, 1 ) = rotZYX.transpose()( 1, 1 );  T( 1, 2 ) = rotZYX.transpose()( 1, 2 );  T( 1, 3 ) = aux( 1, 0 );
        T( 2, 0 ) = rotZYX.transpose()( 2, 0 );  T( 2, 1 ) = rotZYX.transpose()( 2, 1 );  T( 2, 2 ) = rotZYX.transpose()( 2, 2 );  T( 2, 3 ) = aux( 2, 0 );
        T( 3, 0 ) = 0.0;                         T( 3, 1 ) = 0.0;                         T( 3, 2 ) = 0.0;                         T( 3, 3 ) = 1.0;
        //std::cout << "T:\n" << T << "\n\n";

        Eigen::MatrixXd p_req( 4, 1 );
        p_req( 0, 0 ) = setpoint.at( 0 );
        p_req( 1, 0 ) = setpoint.at( 1 );
        p_req( 2, 0 ) = 0.0;
        p_req( 3, 0 ) = 1.0;
        //std::cout << "p_req:\n" << p_req << "\n\n";

        Eigen::MatrixXd distance = T * p_req;
        error.at( 0 ) = distance( 0, 0 );
        error.at( 1 ) = distance( 1, 0 );
        //std::cout << "distance:\n" << distance << "\n\n";
    }


    void
    computeThrusterAllocator()
    {
        if ( _is_thruster_allocator_enable ) {
            _thruster_setpoints = _thruster_allocator.compute( _merged_wrench, _twist_feedback );
        }
    }


    // Must be public to be visible from AUVROSBaseController
    ThrusterAllocatorSPARUS2 _thruster_allocator;
    FinAllocatorSPARUS2 _fin_allocator;

    unsigned int
    getNumberofThrusters() const
    {
        return _n_thrusters;
    }

private:
    unsigned int _n_thrusters;

    // Depth to pitch PID controller
    Pid *_d_to_p_controller;

    // Pitch PID controller
    Pid *_f_roll;
    Pid *_f_pitch;

    // Pose PID controller
    Pid *_p_surge, *_p_sway, *_p_heave, *_p_roll, *_p_pitch, *_p_yaw;
    NDofController _pose_controller;

    // Scale the output of the pose PID controller
    std::vector< double > _twist_factor;

    // Old computed twist value
    std::vector< double > _old_twist;

    // Twist PID controller
    Pid *_t_surge, *_t_sway, *_t_heave, *_t_roll, *_t_pitch, *_t_yaw;
    NDofController _twist_controller;

    // Scale the output of the twist PID controller
    std::vector< double > _force_factor;

    // Twist Poly controller
    Poly *_tp_surge, *_tp_sway, *_tp_heave, *_tp_roll, *_tp_pitch, *_tp_yaw;
    NDofController _twist_poly_controller;

    // Twist Poly percentatge
    std::vector< double > _poly_percentatge;
};

#endif //__SPARUS2_CONTROLLER__
