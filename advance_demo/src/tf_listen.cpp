#include <chrono>
#include <memory>
#include <functional>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include <cmath>


using namespace std::chrono_literals;
using std::placeholders::_1;

class TFListen: public rclcpp::Node
{
public:
  TFListen():Node("tf_listen"){
    RCLCPP_INFO(this->get_logger(), "tf_listen C++ demo");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // 以10Hz的频率循环获取 world - base_link 的最新TF变换
    timer_ = this->create_wall_timer( 100ms, std::bind(&TFListen::listenRecent, this));
  }

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 监听world — base_link的最新TF变换
  void listenRecent(){
    geometry_msgs::msg::TransformStamped trans;
    try {
          trans = tf_buffer_->lookupTransform(
            "world", "base_link",
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "Could not transform world to base_link %s", ex.what());
          return;
        }
      // 平移变换x乘以2,平移变换y乘以2,作为 world—car_1 的平移变换的值
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "car_1";
    t.transform = trans.transform;
    t.transform.translation.x *=2;
    t.transform.translation.y *=2;
      // 广播 world—car_1 的动态TF
    tf_broadcaster_->sendTransform(t);

    geometry_msgs::msg::TransformStamped trans_past;
      try{
        // 监听world — base_link 5s前的TF变换trans_past
        auto past = this->get_clock()->now() - rclcpp::Duration::from_seconds(5.0);
        trans_past = tf_buffer_->lookupTransform(
            "world", "base_link", past);
      }
      catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform world to base_link: %s", ex.what());
      }
      // 将trans_past作为 world—car_2 的平移变换的值,让car_2始终跟随base_link 5s前的位置
      geometry_msgs::msg::TransformStamped t2;
      t2.header.stamp = this->get_clock()->now();
      t2.header.frame_id = "world";
      t2.child_frame_id = "car_2";
      t2.transform = trans_past.transform;
      // 广播 world—car_2 的动态TF
      tf_broadcaster_->sendTransform(t2);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFListen>());
  rclcpp::shutdown();
  return 0;
}

