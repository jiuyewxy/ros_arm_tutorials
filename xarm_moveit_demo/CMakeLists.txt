cmake_minimum_required(VERSION 3.8)
project(xarm_moveit_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)



add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
)

add_executable(moveit_pose_demo src/moveit_pose_demo.cpp)
target_include_directories(moveit_pose_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_pose_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_pose_demo
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
)

add_executable(moveit_joint_pose_demo src/moveit_joint_pose_demo.cpp)
target_include_directories(moveit_joint_pose_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_joint_pose_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_joint_pose_demo
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
)

add_executable(moveit_planning_scene_demo src/moveit_planning_scene_demo.cpp)
target_include_directories(moveit_planning_scene_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_planning_scene_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_planning_scene_demo
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
)

add_executable(moveit_pick_place_demo src/moveit_pick_place_demo.cpp)
target_include_directories(moveit_pick_place_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_pick_place_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_pick_place_demo
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "moveit_task_constructor_core"
)

add_executable(moveit_beeline_demo src/moveit_beeline_demo.cpp)
target_include_directories(moveit_beeline_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_beeline_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
    moveit_beeline_demo
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "moveit_task_constructor_core"
)

add_executable(moveit_arcline_demo src/moveit_arcline_demo.cpp)
target_include_directories(moveit_arcline_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_arcline_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
    moveit_arcline_demo
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "moveit_task_constructor_core"
)

install(TARGETS hello_moveit moveit_pose_demo moveit_joint_pose_demo moveit_planning_scene_demo moveit_pick_place_demo
  moveit_beeline_demo
  moveit_arcline_demo
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
