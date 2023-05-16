#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
int main(int argc, char **argv){
  ros::init(argc, argv, "moveit_arcline_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface arm("xarm");
  arm.allowReplanning(true);
  arm.setMaxVelocityScalingFactor(0.5);

  ROS_INFO("Moving to pose: Home");
  arm.setNamedTarget("Home");
  arm.move();
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x = 0.4;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = 0.45;
  target_pose.pose.orientation.w = 1;
  arm.setStartStateToCurrentState();
  arm.setPoseTarget(target_pose);
  arm.move();
  // 在y-z平面内做圆弧运动,按照0.015弧度对圆进行切分，用一个个线段近似圆弧轨迹
  // y = 圆心的y坐标 + 半径×cos(th)，z = 圆心的z坐标 + 半径×sin(th)
  std::vector<geometry_msgs::Pose> waypoints;
  double centerA = target_pose.pose.position.y;
  double centerB = target_pose.pose.position.z;
  double radius = 0.1;
  for(double th=0;th<=(3.141526*2);th+=0.015){
    target_pose.pose.position.y = centerA + radius * cos(th);
    target_pose.pose.position.z = centerB + radius * sin(th);
    waypoints.push_back(target_pose.pose);
  }
  // 尝试规划一条笛卡尔路径，依次通过所有路点
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("moveit_beeline_demo", "Visualizing plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  // 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
  if(fraction == 1.0){
    arm.execute(trajectory);
  }
  ROS_INFO("Moving to pose: Home");
  arm.setNamedTarget("Home");
  arm.move();
  ros::shutdown();
  return 0;
}
