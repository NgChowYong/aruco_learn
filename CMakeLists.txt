cmake_minimum_required(VERSION 3.0.2)
project(localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  fiducial_msgs
  std_msgs
  roscpp
  rospy
  tf
)

add_message_files(
  FILES
  Camera_Data.msg
  Camera_Data_error.msg
)

add_service_files(
  FILES
  correction_service.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aruco_learn
#  CATKIN_DEPENDS aruco roscpp rospy
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS scripts/correction_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/cam_trans.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )


add_executable(cam_error src/cam_error_measure.cpp)
add_dependencies(cam_error ${cam_error_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cam_error
   ${catkin_LIBRARIES}
 )


add_executable(cam_localization src/camera_localization.cpp)
add_dependencies(cam_localization ${cam_localization_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cam_localization
   ${catkin_LIBRARIES}
 )

