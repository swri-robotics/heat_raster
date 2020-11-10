cmake_minimum_required(VERSION 2.8.3)
project(heat_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  std_srvs
  actionlib
  actionlib_msgs
  message_generation
  pcl_msgs
  shape_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
    Source.msg
    HeatRasterGeneratorConfig.msg
    HeatToolPath.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
    GenerateHeatToolPaths.action
)

## Generate services
add_service_files(
 FILES
    GenerateHeatPaths.srv
)

## Generate dependent messages
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    pcl_msgs
    shape_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    pcl_msgs
    shape_msgs
    std_msgs
)

