#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "base_demo/msg/robot_info.hpp"

using std::placeholders::_1;

class TopicSubscriber : public rclcpp::Node
{
public:
  TopicSubscriber()
  : Node("topic_sub")
  {
    RCLCPP_INFO(this->get_logger(), "topic_sub node is Ready!");
    subscription_ = this->create_subscription<base_demo::msg::RobotInfo>(
      "robot_info", 10, std::bind(&TopicSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const base_demo::msg::RobotInfo & msg) const 
  {
    if(msg.pose.position.x == 20.0){
    RCLCPP_INFO(this->get_logger(), "The robot has reached the target pose.");
      if(msg.is_carry){
        RCLCPP_INFO(this->get_logger(), "The robot is carrying objects");
      }else{
      RCLCPP_INFO(this->get_logger(), "The robot is not carrying objects");
      }
    }else{
      RCLCPP_INFO(this->get_logger(), msg.state.c_str());
      RCLCPP_INFO(this->get_logger(), "Robot pose x : %.2fm; y : %.2fm; z : %.2fm", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    }
  }
  rclcpp::Subscription<base_demo::msg::RobotInfo>::SharedPtr subscription_; 

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicSubscriber>());
  rclcpp::shutdown();
  return 0;
}
