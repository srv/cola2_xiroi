#include "ros/ros.h"
#include "local_planner.h"



// input messages:: octomap 
// 1- create an instance of the state space ompl::base::State *abstractState = space->allocState() 
// 2- convert octomap to state space  ??¿¿
// 2- define state validity checking class -->  bool isStateValid(const ob::State *state)
// 3- set the bounds for the R3 component of this state space
// 4- Create an instance of ompl::geometric::SimpleSetup  --> og::SimpleSetup ss(space);. 
// 5- Set the state validity checker --> ss.setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
// 6- Create a random start state: ob::ScopedState<> start(space); start.random();
// 7- And a random goal state: ob::ScopedState<> goal(space); goal.random(); 
// 8- Set these states as start and goal for SimpleSetup. --> ss.setStartAndGoalStates(start, goal);
// 9- Solve the problem: ob::PlannerStatus solved = ss.solve(1.0); 
// 10- simplify solution: ss.simplifySolution(); ss.getSolutionPath().print(std::cout);

/**  possible values for the variable priority_
 PRIORITY_AVOID_OBSTACLE = 20
 PRIORITY_EMERGENCY = 30
 PRIORITY_MANUAL_OVERRIDE  = 40
# NEW PRIORITY DEFINITIONS
 PRIORITY_TELEOPERATION_LOW = 0
 PRIORITY_SAFETY_LOW = 5
 PRIORITY_NORMAL = 10
 PRIORITY_NORMAL_HIGH = 20
 PRIORITY_TELEOPERATION = 40
 PRIORITY_SAFETY = 45
 PRIORITY_SAFETY_HIGH  = 50
 PRIORITY_TELEOPERATION_HIGH = 60 **/

struct point3D
    {
      double x;
      double y;
      double z; 
    };


class LocalPlanner
{

   double radius_;
   int priority_;
   ros::NodeHandle nh_;
   ros::NodeHandle nh_private_;

   std::string node_name_;

    ros::Subscriber map_sub_;
    ros::Subscriber nav_sub_;
    ros::Subscriber body_vel_sub_;
    ros::Publisher _pub_corrected_velocity;

    point3D vehiclepositionNED; 
    point3D velocity_request_linear;
    point3D velocity_request_angular;
    std::mutex mutex_octomap_;


bool disable_axis_x, disable_axis_y, disable_axis_z, disable_axis_roll, disable_axis_pitch, disable_axis_yaw;

std::vector<point3D> obstacle_point_cloud;
    
public:
  LocalPlanner() : nh_(), nh_private_("~") 
  {

    node_name_ = ros::this_node::getName();
    nh_private_.param("radius", radius_, 3.0);
    nh_private_.param("priority", priority_, 40); 

    vehiclepositionNED.x=0.0;
    vehiclepositionNED.y=0.0;
    vehiclepositionNED.z=0.0;

    //subscribers
    map_sub_ = nh_.subscribe("/cloud", 1, &LocalPlanner::octomapCallback, this);
    nav_sub_ = nh_.subscribe<cola2_msgs::NavSts>("/turbot/navigator/navigation", 1, &LocalPlanner::NavStsCallback, this); 
    body_vel_sub_ = nh_.subscribe<cola2_msgs::BodyVelocityReq>("/turbot/controller/body_velocity_req", 1, &LocalPlanner::BodyVelReqCallback, this);
    //publishers  
    _pub_corrected_velocity = nh_private_.advertise<cola2_msgs::BodyVelocityReq>("/xiroi/controller/body_velocity_req", 2,true );

  }


  void NavStsCallback(const cola2_msgs::NavSts::ConstPtr& nav_sts)
    {
    
  vehiclepositionNED.x = nav_sts->position.north;
  vehiclepositionNED.y = nav_sts->position.east;
  vehiclepositionNED.z = nav_sts->position.depth;

    }

  void BodyVelReqCallback(const cola2_msgs::BodyVelocityReq::ConstPtr& body_velocity_req)
    {

  std::vector<point3D> obstacle_point_cloud_tmp;     
  velocity_request_linear.x = body_velocity_req->twist.linear.x;
  velocity_request_linear.y = body_velocity_req->twist.linear.y;
  velocity_request_linear.z = body_velocity_req->twist.linear.z;
  velocity_request_angular.x=body_velocity_req->twist.angular.x;
  velocity_request_angular.y=body_velocity_req->twist.angular.y;
  velocity_request_angular.z=body_velocity_req->twist.angular.z;
  disable_axis_x= body_velocity_req->disable_axis.x;
  disable_axis_y= body_velocity_req->disable_axis.y;
  disable_axis_z= body_velocity_req->disable_axis.z;
  disable_axis_roll= body_velocity_req->disable_axis.roll;
  disable_axis_pitch= body_velocity_req->disable_axis.pitch;
  disable_axis_yaw= body_velocity_req->disable_axis.yaw; 
 
  mutex_octomap_.lock();
     obstacle_point_cloud_tmp=obstacle_point_cloud;
  mutex_octomap_.unlock();

  correct_body_vel_req(obstacle_point_cloud_tmp); 

    }


  void octomapCallback(const PointCloud::ConstPtr& octomap_point_cloud_centers)
    {
        double distance, dx, dy, dz;
        point3D point;  
        std::vector<point3D> obstacle_point_cloud_tmp;

        for (int i = 0; i < octomap_point_cloud_centers->points.size(); i++)
        {

            const pcl::PointXYZ& pointXYZ = octomap_point_cloud_centers->points[i];

            point.x=pointXYZ.x; 
            point.y=pointXYZ.y;
            point.z=pointXYZ.z;
            dx = (point.x-vehiclepositionNED.x); 
            dy = (point.y-vehiclepositionNED.y);
            dz = (point.z-vehiclepositionNED.z);  
            distance = sqrt( (dx*dx)+ (dy*dy) + (dz*dz) );
            if(distance < radius_){
                obstacle_point_cloud_tmp.push_back(point);
            }

            mutex_octomap_.lock(); 
            obstacle_point_cloud.clear(); 
            obstacle_point_cloud = obstacle_point_cloud_tmp; 
            mutex_octomap_.unlock();
        }
        return;
    }
  void correct_body_vel_req(std::vector<point3D> obstacle_point_cloud_tmp)
  {

    int npt=0, i;
    double mv, dx, dy, dz, ax=0.0, ay=0.0, az=0.0, am=0.0 ; 
    point3D cv, vu, normalized_repulsive_force;
    auv_msgs::BodyVelocityReq  bvr;
        
    for (i=0; i < obstacle_point_cloud_tmp.size(); i++)
        {
            dx = (obstacle_point_cloud_tmp.at(i).x-vehiclepositionNED.x); 
            dy = (obstacle_point_cloud_tmp.at(i).y-vehiclepositionNED.y);
            dz = (obstacle_point_cloud_tmp.at(i).z-vehiclepositionNED.z);  
            ax=ax+dx; ay=ay+dy ; az=az+dz ; 
            npt=npt+1; 
        }

    if (npt>0) 
    {
        am=sqrt( (ax*ax)+ (ay*ay) + (az*az) );   
        normalized_repulsive_force.x = (ax / am) ; 
        normalized_repulsive_force.y = (ay / am) ; 
        normalized_repulsive_force.z = (az / am) ; 
        mv=sqrt((velocity_request_linear.x*velocity_request_linear.x) + (velocity_request_linear.y*velocity_request_linear.y) + (velocity_request_linear.z*velocity_request_linear.z));

        if(mv!=0)
        {
            vu.x=velocity_request_linear.x/mv;
            vu.y=velocity_request_linear.y/mv;
            vu.z=velocity_request_linear.z/mv;
            cv.x = vu.x- normalized_repulsive_force.x;
            cv.y = vu.y- normalized_repulsive_force.y;
            cv.z = vu.z- normalized_repulsive_force.z;
            cv.x = cv.x * mv; 
            cv.y = cv.y * mv;
            cv.z = cv.z * mv;
        }
            else
            {
                vu.x=0;
                vu.y=0;
                vu.z=0;
                cv.x = vu.x- normalized_repulsive_force.x;
                cv.y = vu.y- normalized_repulsive_force.y;
                cv.z = vu.z- normalized_repulsive_force.z;
                cv.x = cv.x; 
                cv.y = cv.y;
                cv.z = cv.z;
            }
        
    }
    else
    {
        ROS_WARN_STREAM("[" << node_name_ << "]: NO OBSTACLE CLOSER THAN !!! " << radius_ << "meters; Publish the requested velocity  " );
        cv.x = velocity_request_linear.x; 
        cv.y = velocity_request_linear.y;
        cv.z = velocity_request_linear.z;
    }

    ros::Time now = ros::Time::now();

    bvr.header.frame_id = node_name_ + "_velocity"; 
    bvr.header.stamp = now;
    bvr.goal.requester = node_name_ + "_velocity";
    bvr.disable_axis.x     = disable_axis_x;
    bvr.disable_axis.y     = disable_axis_y;
    bvr.disable_axis.z     = disable_axis_z;
    bvr.disable_axis.roll  = disable_axis_roll;
    bvr.disable_axis.pitch = disable_axis_pitch;
    bvr.disable_axis.yaw   = disable_axis_yaw;
    bvr.twist.linear.x     = cv.x;
    bvr.twist.linear.y     = cv.y;
    bvr.twist.linear.z     = cv.z;
    bvr.twist.angular.x    = velocity_request_angular.x;
    bvr.twist.angular.y    = velocity_request_angular.y;
    bvr.twist.angular.z    = velocity_request_angular.z;
    bvr.goal.priority      = priority_;
    ROS_WARN_STREAM("[" << node_name_ << "]: Publish velocity" <<  ", " << bvr.twist.linear.x << ", " << bvr.twist.linear.y << ", " << bvr.twist.angular.z );
    _pub_corrected_velocity.publish(bvr);     
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_node");

  LocalPlanner local_planner;

  ros::spin();
  return 0;
}