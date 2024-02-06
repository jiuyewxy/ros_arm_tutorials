#include "rclcpp/rclcpp.hpp"
#include "base_demo/srv/set_target_detec.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv){
  // 初始化ROS节点
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service_client node is Ready!");
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_client");

  rclcpp::Client<base_demo::srv::SetTargetDetec>::SharedPtr client =
    node->create_client<base_demo::srv::SetTargetDetec>("target_detection");
  auto request = std::make_shared<base_demo::srv::SetTargetDetec::Request>();
  request->name = "box";

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto res = result.get();
    if(res->success){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Target detection succeeded!");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot pose x : %.2fm; y : %.2fm; z : %.2fm", res->pose.position.x, res->pose.position.y, res->pose.position.z);
    }else{
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Can not find the target!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service target_detection");
  }

  rclcpp::shutdown();
  return 0;
}
