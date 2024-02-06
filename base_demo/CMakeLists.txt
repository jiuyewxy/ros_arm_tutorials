cmake_minimum_required(VERSION 3.8)
project(base_demo)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotInfo.msg"
  "srv/SetTargetDetec.srv"
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

set(dependencies "ament_index_cpp" "geometry_msgs" "rclcpp" "std_msgs" "std_srvs")
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(topic_pub src/topic_pub.cpp)
target_link_libraries(topic_pub "${cpp_typesupport_target}")
ament_target_dependencies(topic_pub "${dependencies}")

add_executable(topic_sub src/topic_sub.cpp)
target_link_libraries(topic_sub "${cpp_typesupport_target}")
ament_target_dependencies(topic_sub "${dependencies}")    

add_executable(service_server src/service_server.cpp)
target_link_libraries(service_server "${cpp_typesupport_target}")
ament_target_dependencies(service_server "${dependencies}")  

add_executable(service_client src/service_client.cpp)
target_link_libraries(service_client "${cpp_typesupport_target}")
ament_target_dependencies(service_client "${dependencies}")  

add_executable(param_demo src/param_demo.cpp)
target_link_libraries(param_demo "${cpp_typesupport_target}")
ament_target_dependencies(param_demo "${dependencies}")  

install(TARGETS topic_pub topic_sub service_server service_client param_demo
  DESTINATION lib/${PROJECT_NAME})


# Install Python executables
install(PROGRAMS
  scripts/topic_pub.py scripts/topic_sub.py scripts/service_server.py scripts/service_client.py scripts/param_demo.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
