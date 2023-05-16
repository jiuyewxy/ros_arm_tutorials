#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <thread>
#include "advance_demo/PubMarkerConfig.h"

class PubMarker{
public:
  PubMarker():nh_("~")
  {
    ROS_INFO("pub_marker C++ node is Ready!");
    // 设置参数的值
    nh_.param<std::string>("user_name", user_name_, "Xiaode");
    nh_.param<std::string>("marker_frame", marker_frame_, "Xiaode");
    nh_.param<double>("marker_size", marker_size_, 0.2);
    nh_.param<double>("marker_color_r", color_r_, 1);
    nh_.param<double>("marker_color_g", color_g_, 0);
    nh_.param<double>("marker_color_b", color_b_, 0);
    nh_.param<double>("marker_color_a", color_a_, 1);
    nh_.param<double>("speed", speed_, 1.5);
    // 创建动态参数服务端实例,传入定义的配置参数advance_demo::PubMarkerConfig
    server_ = new dynamic_reconfigure::Server<advance_demo::PubMarkerConfig>(ros::NodeHandle("~"));
    // 将回调函数dynamicReconfigureCallback与服务端server_绑定
    dynamic_reconfigure::Server<advance_demo::PubMarkerConfig>::CallbackType cb = boost::bind(&PubMarker::dynamicReconfigureCallback, this, _1, _2);
    server_->setCallback(cb);
    // 创建话题发布端函数
    publisherTopics();
    // 启动新线程用于循环发布Marker
    std::thread pub_topic(&PubMarker::pubMoveMarkers,this);
    pub_topic.detach();
  }

  ~PubMarker(){
    if(server_ != nullptr){
      delete server_;
      server_ = nullptr;
    }
  }

private:
  ros::NodeHandle nh_;
  std::string user_name_,marker_frame_;
  double marker_size_, color_r_, color_g_,color_b_,color_a_,speed_;
  // 声明指向dynamic_reconfigure::Server的指针
  dynamic_reconfigure::Server<advance_demo::PubMarkerConfig> *server_;
  ros::Publisher pose_pub_,marker_pub_,name_pub_;

  // 动态参数配置回调函数.当动态参数更新时,会调用回调函数进行处理
  void dynamicReconfigureCallback(advance_demo::PubMarkerConfig &config, uint32_t level){
    ROS_INFO("Reconfigure Request: %s %s %f %f %f %f %f %f %d",
             config.user_name.c_str(),config.marker_frame.c_str(),config.marker_size,
             config.marker_color_r,config.marker_color_g,config.marker_color_b,
             config.marker_color_a,config.speed,level);
    user_name_ = config.user_name;
    marker_frame_ = config.marker_frame;
    marker_size_ = config.marker_size;
    color_r_ = config.marker_color_r;
    color_g_ = config.marker_color_g;
    color_b_ = config.marker_color_b;
    color_a_ = config.marker_color_a;
    speed_ = config.speed;
  }

  // 创建话题发布端函数
  void publisherTopics(){
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/marker_pose", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/target_marker", 10);
    name_pub_ = nh_.advertise<visualization_msgs::Marker>("/user_name", 10);
  }

  // 设置Marker并循环发布
  void pubMoveMarkers(){
    // 创建Marker对象text_marker和target_marker
    visualization_msgs::Marker text_marker;
    visualization_msgs::Marker target_marker;
    // 创建位姿对象target_pose,用于target_marker的位姿设置
    geometry_msgs::PoseStamped target_pose;

    // 设置text_marker的相关属性
    text_marker.id =1;  // id
    text_marker.header.frame_id = "world";  // 参考系为world
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;  // 类型为可视的有方向的文本
    text_marker.action = visualization_msgs::Marker::MODIFY;  // 行为为更改
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
    target_marker.type = visualization_msgs::Marker::SPHERE;  // 类型为球体
    target_marker.action = visualization_msgs::Marker::ADD;   // 行为为添加
    target_marker.lifetime = ros::Duration();  // marker在RViz中显示的时长，当Duration()无参数时，表示一直显示

    target_pose.pose.orientation.w =1;
    double  rate = 20;
    ros::Rate r(rate);
    double theta = 0;
    // 以20Hz的频率循环向外发布target_marker,text_marker和target_marker的位姿
    while(ros::ok()){
      // target_marker在XY平面上做椭圆运动
      target_pose.header.frame_id = marker_frame_;
      target_pose.pose.position.x = 1.0 + 0.5*sin(theta);
      target_pose.pose.position.z = 0.8 + 0.3*cos(theta);
      theta += speed_/rate;
      ros::Time now = ros::Time::now();
      target_pose.header.stamp = now;
      target_marker.header.stamp = now;
      target_marker.header.frame_id = marker_frame_;
      target_marker.pose.position = target_pose.pose.position;
      target_marker.pose.orientation.w = 1;
      // arget_marker的尺寸颜色可通过动态参数动态设置
      target_marker.scale.x = marker_size_;
      target_marker.scale.y = marker_size_;
      target_marker.scale.z = marker_size_;
      target_marker.color.r = color_r_;
      target_marker.color.g = color_g_;
      target_marker.color.b = color_b_;
      target_marker.color.a = color_a_;
      // 设置text_marker的显示文本
      text_marker.header.stamp = now;
      text_marker.text = "User name is : " + user_name_;
      // 发布话题消息
      pose_pub_.publish(target_pose);
      marker_pub_.publish(target_marker);
      name_pub_.publish(text_marker);
      r.sleep();
    }
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "pub_marker");
  PubMarker pub_marker;
  ros::spin();
  return 0;
}
