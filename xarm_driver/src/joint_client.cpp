#include "ros/ros.h"
#include <xarm_driver/CommandJoint.h>
#include <iostream>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_client");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<xarm_driver::CommandJoint>("xbot_arm/command/joint");
  xarm_driver::CommandJoint srv;
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();

  std::vector<std::string> joints_name = { "arm_1_joint", "arm_2_joint", "arm_3_joint",
                                           "arm_4_joint", "arm_5_joint", "arm_6_joint","gripper_1_joint","gripper_2_joint"};
  std::vector<double> joints_value = {1.2, 0.9, 0.4, 0.9, 1.2, 0.8 ,0 ,0};
  joint_state.header.frame_id = "world";
  joint_state.name = joints_name;
  joint_state.position = joints_value;
  srv.request.JointState = joint_state;
  if(client.call(srv))//调用服务。
  {
    if(srv.response.success){
      ROS_INFO("succed to get the plan");
    }else {
      ROS_INFO("failed to get the plan");

    }
  }
  else
  {
      ROS_INFO("Failed to call service Service_demo_jone");
      return 1;
  }
  return 0;
}

