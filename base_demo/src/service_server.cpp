#include "ros/ros.h"
#include "base_demo/SetTargetDetec.h"
// 服务回调函数,req为服务的请求,res为服务的应答
bool targetDetectionHandle(base_demo::SetTargetDetec::Request &req, base_demo::SetTargetDetec::Response &res){
  // 打印输出req.name
  ROS_INFO_STREAM("Target object is " << req.name);
  ROS_INFO_STREAM("Find " << req.name << " and response.");
  // 对res的成员进行赋值并返回应答
  res.success = true;
  res.pose.position.x = 0.35;
  res.pose.position.y = -0.35;
  res.pose.position.z = 0.1;
  res.pose.orientation.w = 1;
  return true;
}

int main(int argc, char **argv){
  // 初始化ROS节点
  ros::init(argc, argv, "service_server");
  // 创建一个节点句柄（NodeHandle）对象nh
  ros::NodeHandle nh;
  // 创建target_detection服务的服务端对象service,服务回调函数targetDetectionHandle
  ros::ServiceServer service = nh.advertiseService("target_detection", targetDetectionHandle);
  ROS_INFO("Service server is Ready!");
  ros::spin();
  return 0;
}
