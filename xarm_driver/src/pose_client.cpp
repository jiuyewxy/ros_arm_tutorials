#include "ros/ros.h"
#include <xarm_driver/CommandPose.h>
#include <iostream>
#include <geometry_msgs/PointStamped.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_client");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<xarm_driver::CommandPose>("xbot_arm/command/pose");
  xarm_driver::CommandPose srv;
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose.position.x = 0.29;
  pose_stamped.pose.position.y = 0.26;
  pose_stamped.pose.position.z = 0.22;
  pose_stamped.pose.orientation.x=0.067;
  pose_stamped.pose.orientation.y=-0.073;
  pose_stamped.pose.orientation.z=0.7066;
  pose_stamped.pose.orientation.w=0.7005;
  srv.request.PoseStamped = pose_stamped;
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

