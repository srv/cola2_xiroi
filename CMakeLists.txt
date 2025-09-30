cmake_minimum_required(VERSION 3.0.2)
project(cola2_xiroi)

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  diagnostic_aggregator
  joy
  rosbridge_server
  rviz
  tf
  topic_tools

  # COLA2 Dependencies
  xiroi_description
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Current.msg
   EmusBms.msg
   ComputerData.msg
   MinimalDiagnostic.msg
   PowerReading.msg
   ImuAngleEstimatorInfo.msg
 )

 ## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
     std_msgs
#   auv_msgs#   cola2_msgs#   nav_msgs#   sensor_msgs
 )

# Declare things to be passed to dependent projects
catkin_package()

