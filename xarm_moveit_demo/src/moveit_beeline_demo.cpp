#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
int main(int argc, char **argv){
  ros::init(argc, argv, "moveit_beeline_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm("xarm");
  arm.allowReplanning(true);
  arm.setMaxVelocityScalingFactor(0.8);

  ROS_INFO("Moving to pose: Home");
  arm.setNamedTarget("Home");
  arm.move();

  // 设置三角形第一个顶点的位姿，让机械臂从初始状态运动到这个点
  ROS_INFO("Moving to pose: target_pose");
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x = 0.4;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = 0.5;
  target_pose.pose.orientation.w = 1;
  arm.setStartStateToCurrentState();
  arm.setPoseTarget(target_pose);
  arm.move();
  // 获取当前位置并保存到start_pose里，可以用来作为路径点的起点和终点
  geometry_msgs::Pose start_pose = arm.getCurrentPose().pose;
  geometry_msgs::Pose end_pose = start_pose;
  // 初始化路点列表waypoints，用来保存需要使用笛卡尔路径的路径点。
  // 按照顺序依次在路点列表中添加三角形的顶点
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose);
  geometry_msgs::Pose wppose = start_pose;
  wppose.position.z -= 0.2;
  waypoints.push_back(wppose);
  wppose.position.y += 0.2;
  waypoints.push_back(wppose);
  waypoints.push_back(end_pose);
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
