#ifndef __SPARUS2FINALLOCATOR_CLASS__
#define __SPARUS2FINALLOCATOR_CLASS__
#include <algorithm>
#include <map>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <sstream>
#include <cola2_lib/cola2_control/Request.h>

#include <math.h>
#include <complex>


class FinAllocatorSPARUS2 {
public:
    FinAllocatorSPARUS2( unsigned int n_fins ):
                         _n_fins( n_fins ),
                         _old_ret( n_fins ),
                         _is_init( false )
    {
		if ( _n_fins > 0 ) {
			// Load params
			_max_fin_force = 26.15;
			_max_fin_angle = 40;
			_min_fin_angle = -40;
			_fin_distance_surge = 0.65;
			_fin_distance_yaw = 0.14;
			//_force_to_fins_ratio = 10;

			Eigen::MatrixXd fcm( 6, _n_fins );
			fcm << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.14, 0.14, 0.65, 0.65, 0.0, 0.0;
			std::cout << "fcm: \n" << fcm << "\n";

			// Compute pseudoinverse
			_fcm_inv = ( fcm.transpose() * fcm ).inverse() * fcm.transpose();
		}
    }

    ~FinAllocatorSPARUS2()
    {}

    void
    setParams( const double max_fin_force,
               const double fin_distance_surge,
               const double fin_distance_yaw,
               const double max_fin_angle,
               const double min_fin_angle,
               const double force_to_fins_ratio,                                        //*** remove it!
               const std::vector< double > fcm_values )

    {
        std::cout << "FinsAllocatorSPARUS2 set params\n";

        _max_fin_force = max_fin_force;
        _fin_distance_surge = fin_distance_surge;
        _fin_distance_yaw = fin_distance_yaw;
        _max_fin_angle = max_fin_angle;
        _min_fin_angle = min_fin_angle;
        //_force_to_fins_ratio = force_to_fins_ratio;

        // Init FCM inverse
        std::cout << "fcm_values.size(): " << fcm_values.size() << "\n";
        assert(fcm_values.size() == 6*_n_fins);
        Eigen::MatrixXd fcm( 6, _n_fins );
        for( unsigned int i = 0; i < 6; i++ ) {
            for( unsigned int j = 0; j < _n_fins; j++ ) {
                fcm(i, j) = fcm_values.at( i*_n_fins + j );
            }
        }

        std::cout << "FCM:\n"<< fcm << "\n";
        _fcm_inv = ( fcm.transpose() * fcm ).inverse() * fcm.transpose();
        std::cout << "FCM inv:\n" << _fcm_inv << "\n";

        std::cout << "FinsAllocatorSPARUS2 initialized!\n";
        _is_init = true;
    }

    Eigen::VectorXd
    calibration( double fins_setpoint )
    {
        Eigen::VectorXd setpoint( 2 );
        setpoint[0] = fins_setpoint * 5/0.0349;
        setpoint[1] = fins_setpoint * 5/0.0349;
        std::cout << "Fins setpoint: \n" << setpoint << "\n";

        // Publish
        return setpoint;
    }

    Eigen::VectorXd
    compute( const Request& wrench, const Eigen::VectorXd& th_set, const std::vector<double>& twist_feedback )
    {
        if( !_is_init ) return Eigen::MatrixXd::Zero( _n_fins, 1 );

        // Take wrench request if disabled false, otherwise, get 0.0
        //std::cout << "fin allocator\n" << wrench << "\n";
        Eigen::VectorXd wrench_req( 6 );
        for( unsigned int i = 0; i < wrench.getDisabledAxis().size(); i++ ) {
            if( wrench.getDisabledAxis().at(i) ) {
                wrench_req[i] = 0.0;
            }
            else {
                wrench_req[i] = wrench.getValues().at(i);
            }
        }

        // Merge Roll and Pitch
        /*double roll = wrench_req[3];
        double pitch = wrench_req[4];
        mergeRollPitch( roll, pitch );
        wrench_req[3] = roll;
        wrench_req[4] = pitch;*/
        mergeRollPitch( wrench_req[3], wrench_req[4] );
        //std::cout << "wrench:\n" << wrench_req << "\n";

        // Multiply wrench by fin allocation matrix
        Eigen::VectorXd force_per_fin;
        force_per_fin = _fcm_inv * wrench_req;
        //std::cout << "force_per_fin: \n" << force_per_fin << "\n";

        // Get the setpoints of the horizontal thrusters                                    TODO: do it with a matrix in the .yaml
        Eigen::VectorXd thruster_setpoint( 3 );
        thruster_setpoint[1] = th_set[1]; //right_th
        thruster_setpoint[0] = th_set[2];  //left_th
        thruster_setpoint[2] = th_set[0];

        std::vector<double> twist ( 6 );
        twist[0] = twist_feedback[0];
        twist[2] = twist_feedback[2];
        //std::cout << "Left thruster: " << horizontal_thruster_setpoint[0] << "\n";
        //std::cout << "Right thruster: " << horizontal_thruster_setpoint[1] << "\n";

        // Force to setpoint
        Eigen::VectorXd setpoint = forceToSetpoint( force_per_fin, thruster_setpoint, twist );
        //std::cout << "setpoint: \n" << setpoint << "\n";

        // Publish
        return setpoint;
    }


private:
    unsigned int _n_fins;
    double _max_fin_force;
    double _fin_distance_surge;
    double _fin_distance_yaw;
    double _max_fin_angle;
    double _min_fin_angle;
    //double _force_to_fins_ratio;
    Eigen::MatrixXd _fcm_inv;
    Eigen::VectorXd _old_ret;
    bool _is_init;



    Eigen::VectorXd
    forceToSetpoint( Eigen::VectorXd& wrench, const Eigen::VectorXd& thruster_setpoints, const std::vector< double >& twist )
    {
        Eigen::VectorXd ret( wrench.size() );
                                                                                        //TODO: assert that fins size is equal to horizontal_th size

        // Compute newtons to setpoints for each fin
        for( unsigned int i = 0; i < wrench.size(); i++ ) {
            //std::cout << "ret " << i << ": " << ret[i] << "\n";

            // Checking the wrench limit
            saturate( wrench[i], 26, -26 );

            // Initialize setpoint
            ret[i] = 0.0;

            if ( twist.at(0) < 0.05 and twist.at(0) > -0.1 and twist.at(2) > 0.1 and thruster_setpoints[i] < 0.15 ) {

                // With + heave set fins to +60 degrees
                ret[i] = 60.0;
                std::cout << "TO MAX !!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
            }
            else if ( twist.at(0) < 0.05 and twist.at(0) > -0.1 and twist.at(2) < -0.1 and thruster_setpoints[i] < 0.15 ) {

                // With - heave set fins to -60 degrees
                ret[i] = -60.0;
                std::cout << "TO MIN !!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
            }
            else { // Apply fins model

                // Ensure the output angle if desired torque is 0 despite model inacuracies
                if ( wrench[i] == 0 ) {
                    ret[i] = 0;
                }
                else {
                    bool saturate_setpoint = false;

                    // Ensure the inputs of the model inside the limits
                    if ( ( thruster_setpoints[i] > 0 ) and ( thruster_setpoints[i] <= 1 ) ) {
                        if ( fabs( wrench[i] ) < _max_fin_force )  {

                            // Fins model -> angle(force_per_fin,th_setpoint): Output in degrees
                            std::complex<double> angle = pow( 268140.86 * pow ( thruster_setpoints[i], 2 ) - 21481.84 * thruster_setpoints[i] + 1544.68 * -fabs(wrench[i]) + 832.42 , 0.5 ) - 517.82 * thruster_setpoints[i] + 19.51;

                            // Avoiding bad values due to computations out of the model
                            if ( ( std::isfinite( std::real( angle ) ) ) and ( std::imag( angle ) == 0 ) ) {
                                // Avoiding bad values ( the model has to give negative angles )
                                if ( std::real( angle ) < 0.0 ) {
                                    // At this point the given angle value for the model is ok, just update the angle sign depending on the wrench request
                                    //std::cout << i << " Force: " << -fabs(wrench[i]) << " Setpoints: " << thruster_setpoints[i] << " Angle: " << angle << "\n";
                                    if ( wrench[i] < 0 ) {
                                        ret[i] = std::real( angle );
                                    }
                                    else if ( wrench[i] > 0 ) {
                                        ret[i] = -std::real( angle );
                                    }
                                }
                                else {
                                    // Set to zero degrees if there is any inaccuracy in the model
                                    ret[i] = 0;
                                }
                            }
                            else {
                                // If there is imaginary part the model has computed too small thruster setpoint for the asked force, so
                                // the output angle is saturated
                                saturate_setpoint = true;
                            }
                        }
                        else {
                            //If more force is asked, saturate
                            saturate_setpoint = true;
                        }

                        // Saturate the value if it is required
                        if ( saturate_setpoint ) {
                            if ( wrench[i] < 0 ) {
                                ret[i] = _min_fin_angle;
                            }
                            else if ( wrench[i] > 0 ) {
                                ret[i] = _max_fin_angle;
                            }
                        }
                    }
                    else {
                        // Set to zero degrees if the model can not computed
                        ret[i] = 0;
                    }
                }
                 //std::cout << "ret normalized " << i << ": " << ret[i] << "\n";

                // Saturate between fins angle limits
                saturate ( ret[i], _max_fin_angle, _min_fin_angle );
                //std::cout << "saturated ret " << i << ": " << ret[i] << "\n";
            }
        }
        return ret;
    }

    void
    mergeRollPitch( double& roll, double& pitch ) {
        // If the composition of Pitch and Roll torques override the maximum force
        // per fin, Roll torque is respected and Pitch torque is reduced

        // Saturate roll torque
        //std::cout << "Roll input: " << roll << "\n";
        saturate( roll, 2 * _max_fin_force * _fin_distance_yaw, - 2 * _max_fin_force * _fin_distance_yaw );
        //std::cout << "Saturated roll: " << roll << "\n";

        // Saturate pitch torque
        //std::cout << "Pitch input: " << pitch << "\n";
        saturate( pitch, 2 * _max_fin_force * _fin_distance_surge, - 2 * _max_fin_force * _fin_distance_surge );
        //std::cout << "Saturated pitch: " << pitch << "\n";

        double l_fin = 0.0;
        double r_fin = 0.0;

        if ( roll != 0.0 ) {
            l_fin = l_fin - ( ( roll / _fin_distance_yaw ) / 2 );
            r_fin = r_fin + ( ( roll / _fin_distance_yaw ) / 2 );
        }

        if ( pitch != 0 ) {
            l_fin = l_fin + ( ( pitch / _fin_distance_surge ) / 2 );
            r_fin = r_fin + ( ( pitch / _fin_distance_surge ) / 2 );
        }

        // Check limits
        double diff = 0.0;

        if ( fabs( l_fin ) > _max_fin_force ) {
            diff = l_fin - ( fabs( l_fin ) / l_fin ) * _max_fin_force;
        }

        if ( fabs( r_fin ) > _max_fin_force ) {
            diff = r_fin - ( fabs( r_fin ) / r_fin ) * _max_fin_force;
        }

        if ( diff != 0.0 ) {
            l_fin = l_fin - diff;
            r_fin = r_fin - diff;
        }

        //std::cout << "Left fin force: " << l_fin << "\n";
        //std::cout << "Right fin force: " << r_fin << "\n";

        // Compose again pitch and roll torques
        roll = ( -l_fin + r_fin ) * _fin_distance_yaw;
        pitch = ( l_fin + r_fin ) * _fin_distance_surge;
    }

    void
    saturate( double& value,
              const double max_value,
              const double min_value )
    {
        if( value > max_value ) value = max_value;
        if( value < min_value ) value = min_value;
    }
};

#endif // __SPARUS2FINALLOCATOR_CLASS__
