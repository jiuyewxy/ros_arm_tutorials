cmake_minimum_required(VERSION 3.8)
project(advance_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickupPlace.action"
  DEPENDENCIES std_msgs geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
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

set(dependencies "ament_index_cpp" "geometry_msgs" "visualization_msgs" "tf2" "tf2_ros" "rclcpp" "std_msgs" "rclcpp_action" "rclcpp_components")
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(action_server src/action_server.cpp)
target_link_libraries(action_server "${cpp_typesupport_target}")
ament_target_dependencies(action_server "${dependencies}")

add_executable(action_client src/action_client.cpp)
target_link_libraries(action_client "${cpp_typesupport_target}")
ament_target_dependencies(action_client "${dependencies}")

add_executable(pub_marker src/pub_marker.cpp)
target_link_libraries(pub_marker "${cpp_typesupport_target}")
ament_target_dependencies(pub_marker "${dependencies}")

add_executable(tf_pub src/tf_pub.cpp)
target_link_libraries(tf_pub "${cpp_typesupport_target}")
ament_target_dependencies(tf_pub "${dependencies}")

add_executable(tf_listen src/tf_listen.cpp)
target_link_libraries(tf_listen "${cpp_typesupport_target}")
ament_target_dependencies(tf_listen "${dependencies}")


install(TARGETS action_server action_client pub_marker tf_pub tf_listen
  DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/action_server.py   scripts/action_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
