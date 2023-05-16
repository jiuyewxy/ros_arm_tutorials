cmake_minimum_required(VERSION 2.8.3)
project(xarm_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ecl_build
  ecl_devices
  ecl_geometry
  ecl_time
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
  geometry_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  serial
  moveit_core
  #moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorStatus.msg
  SingleJointControl.msg
  JointLocation.msg
)

## Generate services in the 'srv' folder
 add_service_files(
     FILES
     CommandPose.srv
     CommandJoint.srv
     CallVersion.srv
 )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
	#  LIBRARIES xarm_driver
  CATKIN_DEPENDS ecl_build ecl_devices ecl_geometry ecl_time roscpp rospy std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(xarm_driver
    src/xarm_ros.cpp
    src/xarm_ros_wrapper.cpp
    src/xarm_action.cpp
    src/xarm_driver.cpp
    src/command.cpp
    src/packet/motor_status.cpp
    src/packet_handler/packet_finder.cpp
)

add_dependencies(xarm_driver  xarm_driver_gencpp ${xarm_driver_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xarm_driver ${catkin_LIBRARIES} )



add_executable(arm_node src/main.cpp src/packet/motor_status.cpp
    src/packet_handler/packet_finder.cpp   src/command.cpp  src/xarm_driver.cpp   src/xarm_ros_wrapper.cpp
)
target_link_libraries(arm_node  ${catkin_LIBRARIES})
add_dependencies(arm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(arm_base_api src/basic_api.cpp)
target_link_libraries(arm_base_api  ${catkin_LIBRARIES})
add_dependencies(arm_base_api ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pose_client src/pose_client.cpp)
target_link_libraries(pose_client  ${catkin_LIBRARIES})
add_dependencies(pose_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joint_client src/joint_client.cpp)
target_link_libraries(joint_client  ${catkin_LIBRARIES})
add_dependencies(joint_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS arm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


