#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class PubMarker : public rclcpp::Node
{
public:
  PubMarker()
  : Node("pub_marker"), theta(0)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "pub_marker C++ node is Ready!");
    // 设置参数的值
    this->declare_parameter<std::string>("user_name","Xiaode");
    user_name_ = this->get_parameter("user_name").as_string();
    this->declare_parameter<std::string>("marker_frame","world");
    marker_frame_ = this->get_parameter("marker_frame").as_string();
    this->declare_parameter("marker_size",0.3);
    marker_size_ = this->get_parameter("marker_size").as_double();
    this->declare_parameter("speed",1.5);
    speed_ = this->get_parameter("speed").as_double();

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/marker_pose", 10);  
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/target_marker", 10);  
    name_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/user_name", 10);  
    
    // 以20Hz的频率循环向外发布target_marker,text_marker和target_marker的位姿
    timer_ = this->create_wall_timer(
      50ms, std::bind(&PubMarker::pubMoveMarkers, this));
    // Setup callback for changes to parameters.
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this->get_node_base_interface(),
      this->get_node_topics_interface(),
      this->get_node_graph_interface(),
      this->get_node_services_interface());
    parameter_event_sub_ = parameters_client_->on_parameter_event(std::bind(&PubMarker::onParameterEventCallback, this, std::placeholders::_1));

  }

  ~PubMarker(){ }

private:
  std::string user_name_,marker_frame_;
  double marker_size_,speed_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_; 
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;             
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr name_pub_;  
  double theta;
  // Subscription for parameter change
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  void onParameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event){
  
  for (auto & changed_parameter : event->changed_parameters) {
    const auto & type = changed_parameter.value.type;
    const auto & name = changed_parameter.name;
    const auto & value = changed_parameter.value;

    if (type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      if (name == "user_name") {
        user_name_ = value.string_value;
        RCLCPP_INFO(this->get_logger(), "update user_name: %s ", user_name_.c_str());
      }
    }
    else if(type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
    {
      if (name == "speed") {
        speed_ = value.double_value;//不会改变定时器执行频率，仅作为演示用
        RCLCPP_INFO(this->get_logger(), "update speed: %f ", speed_);
      }
    }
  }
  }

  // 设置Marker并循环发布
  void pubMoveMarkers(){
    // 创建Marker对象text_marker和target_marker
    visualization_msgs::msg::Marker text_marker;
    visualization_msgs::msg::Marker target_marker;
    // 创建位姿对象target_pose,用于target_marker的位姿设置
    geometry_msgs::msg::PoseStamped target_pose;

    // 设置text_marker的相关属性
    text_marker.id =1;  // id
    text_marker.header.frame_id = "world";  // 参考系为world
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;  // 类型为可视的有方向的文本
    text_marker.action = visualization_msgs::msg::Marker::MODIFY;  // 行为为更改
    // 位姿Pose
    text_marker.pose.position.z = 2;
    text_marker.pose.orientation.w =1;
    // 尺寸scale
    text_marker.scale.x = 0.5;
    text_marker.scale.y = 0.5;
    text_marker.scale.z = 0.5;
    // 颜色color
    text_marker.color.b = 1;
    text_marker.color.a = 1;

    // 设置target_marker的相关属性
    target_marker.id = 0;
    target_marker.type = visualization_msgs::msg::Marker::SPHERE;  // 类型为球体
    target_marker.action = visualization_msgs::msg::Marker::ADD;   // 行为为添加
    //target_marker.lifetime = ros::Duration();  // marker在RViz中显示的时长，当Duration()无参数时，表示一直显示

    target_pose.pose.orientation.w =1;
    double  rate = 20;
    // target_marker在XY平面上做椭圆运动
    target_pose.header.frame_id = marker_frame_;
    target_pose.pose.position.x = 1.0 + 0.5*sin(theta);
    target_pose.pose.position.z = 0.8 + 0.3*cos(theta);
    theta += speed_/rate;
    //ros::Time now = ros::Time::now();
    //target_pose.header.stamp = now;
    //target_marker.header.stamp = now;
    target_marker.header.frame_id = marker_frame_;
    target_marker.pose.position = target_pose.pose.position;
    target_marker.pose.orientation.w = 1;
    // arget_marker的尺寸颜色可通过动态参数动态设置
    target_marker.scale.x = marker_size_;
    target_marker.scale.y = marker_size_;
    target_marker.scale.z = marker_size_;
    target_marker.color.r = 0;
    target_marker.color.g = 1;
    target_marker.color.b = 0;
    target_marker.color.a = 1;
    // 设置text_marker的显示文本
    //text_marker.header.stamp = now;
    text_marker.text = "User name is : " + user_name_;
    // 发布话题消息
    pose_pub_->publish(target_pose);
    marker_pub_->publish(target_marker);
    name_pub_->publish(text_marker);
  }
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubMarker>());
  rclcpp::shutdown();
  return 0;
}
