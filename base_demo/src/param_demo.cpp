#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("param_demo");

  // 设置参数
  node->declare_parameter("/a_string", "hello word");
  std::vector<double> vector_param = {1.0, 2.1, 3.2, 4.3};
  node->declare_parameter("/vector_of_double", vector_param);
  node->declare_parameter("relative_int", 2);
  node->declare_parameter("bool_True", true);

  // 获取参数的值
  std::string string_param = node->get_parameter("/a_string").as_string();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Param /a_string : "<<string_param);
  auto vector_param_value = node->get_parameter("/vector_of_double");
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Param /vector_of_double :  "<<vector_param_value);
  return 0;
}
