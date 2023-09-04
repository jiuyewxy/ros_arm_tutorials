cmake_minimum_required(VERSION 3.0.2)
project(xarm_moveit_demo)

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
  #tf2
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
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
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(moveit_joint_pose_demo src/moveit_joint_pose_demo.cpp)
target_link_libraries(moveit_joint_pose_demo ${catkin_LIBRARIES})
add_dependencies(moveit_joint_pose_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(moveit_pose_demo src/moveit_pose_demo.cpp)
target_link_libraries(moveit_pose_demo ${catkin_LIBRARIES})
add_dependencies(moveit_pose_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(moveit_beeline_demo src/moveit_beeline_demo.cpp)
target_link_libraries(moveit_beeline_demo ${catkin_LIBRARIES})
add_dependencies(moveit_beeline_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(moveit_arcline_demo src/moveit_arcline_demo.cpp)
target_link_libraries(moveit_arcline_demo ${catkin_LIBRARIES})
add_dependencies(moveit_arcline_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(moveit_planning_scene_demo src/moveit_planning_scene_demo.cpp)
target_link_libraries(moveit_planning_scene_demo ${catkin_LIBRARIES})
add_dependencies(moveit_planning_scene_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(moveit_pick_place_demo src/moveit_pick_place_demo.cpp)
target_link_libraries(moveit_pick_place_demo ${catkin_LIBRARIES})
add_dependencies(moveit_pick_place_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
        scripts/moveit_arcline_demo.py scripts/moveit_beeline_demo.py scripts/moveit_joint_pose_demo.py scripts/moveit_pick_place_demo.py
        scripts/moveit_planning_scene_demo.py scripts/moveit_pose_demo.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
