cmake_minimum_required(VERSION 3.0.2)
project(cola2_xiroi)

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  diagnostic_aggregator
  joy
  rosbridge_server
  rviz
  tf
  topic_tools
  cola2_msgs

  # COLA2 Dependencies
  xiroi_description
  cola2_lib_ros
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

 ## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetNedOrigin.srv
  RecordRequest.srv
  EnableNavStatus.srv
  SharePosition.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
     std_msgs
#   auv_msgs#   cola2_msgs#   nav_msgs#   sensor_msgs
 )

# Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(nav_status src/nav_status.cpp)
target_link_libraries(nav_status ${catkin_LIBRARIES})
add_dependencies(nav_status ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dummy_to_compare src/dummy_to_compare.cpp)
target_link_libraries(dummy_to_compare ${catkin_LIBRARIES})
add_dependencies(dummy_to_compare ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
