#include <memory>
#include <sstream>
#include <functional>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
using std::placeholders::_1;

class TFPublish : public rclcpp::Node{
public:
  TFPublish(): Node("tf_pub"){
    RCLCPP_INFO_STREAM(this->get_logger(), "tf_pub C++ demo.");
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // Publish static transforms once at startup
    this->pub_static_transforms();

    // 创建/tf_pub/marker_pose话题的订阅端，回调处理函数为poseCB
    marker_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tf_pub/marker_pose", 10, std::bind(&TFPublish::poseCB, this, _1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_sub_; 

  void pub_static_transforms()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "head_link";
    // 设置父坐标系到子坐标系的平移变换
    t.transform.translation.x = 0.1;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.3;
    // 设置父坐标系到子坐标系的旋转变换(四元数表示)
    tf2::Quaternion q;
    q.setRPY(0,0,0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    // 广播 base_link — head_link 的静态TF
    tf_static_broadcaster_->sendTransform(t);
  };

  // /marker_pose话题回调处理函数
  void poseCB(const geometry_msgs::msg::PoseStamped & msg) {
    //  创建TransformStamped对象dynamic_tf_1: world — base_link
    geometry_msgs::msg::TransformStamped dynamic_tf_1;
    dynamic_tf_1.header.stamp = this->get_clock()->now();
    dynamic_tf_1.header.frame_id = "world";
    dynamic_tf_1.child_frame_id = "base_link";
    // 平移变换随Marker变化，base_link始终位于Marker下方
    dynamic_tf_1.transform.translation.x = msg.pose.position.x;
    dynamic_tf_1.transform.translation.y = msg.pose.position.y;
    dynamic_tf_1.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0,0,0);
    dynamic_tf_1.transform.rotation.x = q.x();
    dynamic_tf_1.transform.rotation.y = q.y();
    dynamic_tf_1.transform.rotation.z = q.z();
    dynamic_tf_1.transform.rotation.w = q.w();
    // 广播 world — base_link 的动态TF
    tf_broadcaster_->sendTransform(dynamic_tf_1);

    // 创建TransformStamped对象dynamic_tf_2: head_link — camera_link
    // camera_link的俯仰角随Marker位置变化，Z轴始终指向Marker中心
    geometry_msgs::msg::TransformStamped dynamic_tf_2;
    dynamic_tf_2.header.stamp = this->get_clock()->now();
    dynamic_tf_2.header.frame_id = "head_link";
    dynamic_tf_2.child_frame_id = "camera_link";
    dynamic_tf_2.transform.translation.x = 0;
    dynamic_tf_2.transform.translation.y = 0;
    dynamic_tf_2.transform.translation.z = 0.08;
    double pitch = std::atan((msg.pose.position.z - 0.08 - 0.3)/ 0.1) - 3.1415926/2.0;
    q.setRPY(0,pitch,0);
    dynamic_tf_2.transform.rotation.x = q.x();
    dynamic_tf_2.transform.rotation.y = q.y();
    dynamic_tf_2.transform.rotation.z = q.z();
    dynamic_tf_2.transform.rotation.w = q.w();
    // 发布 head_link — camera_link 的动态TF
    tf_broadcaster_->sendTransform(dynamic_tf_2);
  }

};
int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFPublish>());
  rclcpp::shutdown();
  return 0;
}
