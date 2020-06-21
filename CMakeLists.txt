cmake_minimum_required(VERSION 3.0.2)
project(localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  fiducial_msgs
  std_msgs
  roscpp
  rospy
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aruco_learn
#  CATKIN_DEPENDS aruco roscpp rospy
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/cam_trans.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

