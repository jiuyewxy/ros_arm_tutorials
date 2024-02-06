#include "rclcpp/rclcpp.hpp"
#include "base_demo/srv/set_target_detec.hpp"

// 服务回调函数,req为服务的请求,res为服务的应答
void targetDetectionHandle(const std::shared_ptr<base_demo::srv::SetTargetDetec::Request> req,
          std::shared_ptr<base_demo::srv::SetTargetDetec::Response>   res){
  // 打印输出req.name
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Target object is " << req->name);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Find " << req->name << " and response.");
  // 对res的成员进行赋值并返回应答
  res->success = true;
  res->pose.position.x = 0.35;
  res->pose.position.y = -0.35;
  res->pose.position.z = 0.1;
  res->pose.orientation.w = 1;
}

int main(int argc, char **argv){
  // 初始化ROS节点
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_server");

  rclcpp::Service<base_demo::srv::SetTargetDetec>::SharedPtr service =
    node->create_service<base_demo::srv::SetTargetDetec>("target_detection", &targetDetectionHandle);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service server is Ready!");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
