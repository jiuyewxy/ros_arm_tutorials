#include "ros/ros.h"
#include "base_demo/RobotInfo.h"

int main(int argc, char **argv){
  // 初始化ROS节点
  ros::init(argc, argv, "topic_pub");
  // 创建一个节点句柄（NodeHandle）对象nh
  ros::NodeHandle nh;
  // 打印输出日志消息
  ROS_INFO("topic_pub node is Ready!");
  // 创建Publisher对象pub,用于向话题/robot_info发布RobotInfo消息
  ros::Publisher pub = nh.advertise<base_demo::RobotInfo>("/robot_info", 10);
  // 创建了RobotInfo的对象msg，并对msg里面的部分成员进行赋值
  base_demo::RobotInfo msg;
  msg.is_carry = false;
  msg.header.frame_id = "map";
  msg.pose.position.x = 0;
  msg.pose.orientation.w =1;
  // 创建rate对象,设置频率为5Hz,用于循环发布
  ros::Rate rate(5);
  bool go_flag = true;
  // 循环发布消息
  while(ros::ok()){
    // 设置state并改变msg中机器人在X方向上的位置
    msg.state = "Robot is moving...";
    if(msg.pose.position.x == 0.0){
      go_flag = true;
    }
    if(msg.pose.position.x == 20.0){
      go_flag = false;
    }
    if(go_flag){
      msg.pose.position.x += 0.5;
    }else{
      msg.pose.position.x -= 0.5;
    }
    // header的时间戳为当前时间
    msg.header.stamp = ros::Time::now();
    // 打印输出机器人X方向的位置
    ROS_INFO("Robot pose x : %.1fm", msg.pose.position.x);
    // 发布消息
    pub.publish(msg);
    // 按照循环频率延时
    rate.sleep();
  }

  return 0;
}
