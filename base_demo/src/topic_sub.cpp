#include "ros/ros.h"
#include "base_demo/RobotInfo.h"
// 话题回调函数的其他参数形式
// void callBack(const base_demo::RobotInfoConstPtr &msg)
// void callBack(const base_demo::RobotInfo::ConstPtr &msg)
// void callBack(boost::shared_ptr<base_demo::RobotInfo const> msg)
// void callBack(base_demo::RobotInfo::ConstPtr msg)
// void callBack(base_demo::RobotInfoConstPtr msg)
// void callBack(const base_demo::RobotInfo &msg)
// void callBack(base_demo::RobotInfo msg)
// void callBack(const ros::MessageEvent<base_demo::RobotInfo const> &msg)

// 接收到消息后,进入该回调函数
void callBack(const boost::shared_ptr<base_demo::RobotInfo const> &msg){
  if(msg->pose.position.x == 20.0){
    ROS_INFO("The robot has reached the target pose.");
    if(msg->is_carry){
      ROS_INFO("The robot is carrying objects");
    }else{
      ROS_INFO("The robot is not carrying objects");
    }
  }else{
    ROS_INFO_STREAM(msg->state);
    ROS_INFO("Robot pose x : %.2fm; y : %.2fm; z : %.2fm", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }
}

int main(int argc, char **argv){
  // 初始化节点
  ros::init(argc, argv, "topic_sub", ros::init_options::AnonymousName);
  // 创建一个节点句柄（NodeHandle）对象nh
  ros::NodeHandle nh;
  // 打印输出日志消息
  ROS_INFO("topic_sub node is Ready!");
  // 创建/robot_info话题的Subscriber对象,话题的回调处理函数为callBack
  ros::Subscriber sub = nh.subscribe("robot_info", 10, callBack);
  // 循环等待回调函数
  ros::spin();
  return 0;
}
