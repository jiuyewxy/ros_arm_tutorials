#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "base_demo/msg/robot_info.hpp"

using namespace std::chrono_literals;

class TopicPublisher : public rclcpp::Node
{
public:
  TopicPublisher()
  : Node("topic_pub"), count_(0)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "topic_pub node is Ready!");
    go_flag_ =true;
    message_.is_carry = false;
    message_.header.frame_id = "map";
    message_.pose.position.x = 0;
    message_.pose.orientation.w =1; 
    message_.state = "Robot is moving...";
    publisher_ = this->create_publisher<base_demo::msg::RobotInfo>("/robot_info", 10);  
    timer_ = this->create_wall_timer(
      200ms, std::bind(&TopicPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if(message_.pose.position.x == 0.0){
      go_flag_ = true;
    }
    if(message_.pose.position.x == 20.0){
      go_flag_ = false;
    }
    if(go_flag_){
      message_.pose.position.x += 0.5;
    }else{
      message_.pose.position.x -= 0.5;
    }                                 
    RCLCPP_INFO(this->get_logger(), "Robot pose x : %.1fm", message_.pose.position.x);    
    publisher_->publish(message_);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<base_demo::msg::RobotInfo>::SharedPtr publisher_;             
  size_t count_;
  base_demo::msg::RobotInfo message_;
  bool go_flag_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicPublisher>());
  rclcpp::shutdown();
  return 0;
}

