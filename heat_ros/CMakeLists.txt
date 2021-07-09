cmake_minimum_required(VERSION 2.8.3)

project(heat_ros)

add_compile_options(-std=c++14)

find_package(libgeodesic REQUIRED)
find_package(geometry-central REQUIRED)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  roscpp
  heat_msgs
  smooth_pose_traj
  pcl_ros
  visualization_msgs
  geometric_shapes
)

find_package(PCL 1.9 REQUIRED COMPONENTS visualization)

find_package(VTK REQUIRED NO_MODULE)
if(VTK_FOUND AND ("${VTK_VERSION}" VERSION_LESS 7.1))
  message(FATAL_ERROR "The minimum required version of VTK is 7.1, but found ${VTK_VERSION}")
  set(VTK_FOUND FALSE)
else()
  include(${VTK_USE_FILE})
endif()

catkin_package(
  INCLUDE_DIRS
    include
    ${libgeodesic_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS
    actionlib
    heat_msgs
    roscpp
    smooth_pose_traj
    pcl_ros
    pcl
    visualization_msgs
    geometric_shapes
  DEPENDS
    VTK
    libgeodesic
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LAPACK_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_executable(heat_surface_plan_server
  src/heat_surface_plan_server.cpp
  src/heat_surface_planner.cpp
)

target_link_libraries(heat_surface_plan_server
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  libgeodesic::libgeodesic
  libgeodesic::libgeodesic_cpp
  geometry-central::geometry-central
)

add_executable(pcl_pc_to_mesh
    src/pcl_pc_to_mesh.cpp
)

target_link_libraries(pcl_pc_to_mesh
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS  heat_surface_plan_server pcl_pc_to_mesh
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

