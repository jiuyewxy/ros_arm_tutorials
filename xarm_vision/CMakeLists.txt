cmake_minimum_required(VERSION 3.0.2)
project(xarm_vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  dynamic_reconfigure
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(OpenCV)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


# Generate services in the 'srv' folder
 add_service_files(
   FILES
   CallPickPlaceDemo.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/HSVColorDetection.cfg
  cfg/SetTFOffset.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xarm_vision
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs dynamic_reconfigure message_runtime
  DEPENDS system_lib EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cv_bridge_demo src/cv_bridge_demo.cpp)
target_link_libraries(cv_bridge_demo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(cv_bridge_demo ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(color_detection_demo src/color_detection_demo.cpp)
target_link_libraries(color_detection_demo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(color_detection_demo ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pick_with_AR_server src/pick_with_AR_server.cpp)
target_link_libraries(pick_with_AR_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(pick_with_AR_server ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


catkin_install_python(PROGRAMS
        scripts/color_detection_demo.py scripts/cv_bridge_demo.py scripts/pick_with_AR_server.py scripts/pub_camera_TF.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )