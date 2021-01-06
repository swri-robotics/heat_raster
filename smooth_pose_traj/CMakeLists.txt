cmake_minimum_required(VERSION 3.0.2)
project(smooth_pose_traj)

add_compile_options(-std=c++14)

find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  std_srvs
  message_generation
)

add_definitions(${PCL_DEFINITIONS})


add_service_files(
   FILES
     SmoothPoseTrajectory.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    roscpp
    std_msgs
    std_srvs
  DEPENDS
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/smooth_pose_traj.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

add_executable(smooth_pose_traj_srv src/smooth_pose_traj_srv.cpp)
add_dependencies(smooth_pose_traj_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(smooth_pose_traj_srv ${PROJECT_NAME} ${catkin_LIBRARIES} )

install(TARGETS ${PROJECT_NAME} smooth_pose_traj_srv
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

