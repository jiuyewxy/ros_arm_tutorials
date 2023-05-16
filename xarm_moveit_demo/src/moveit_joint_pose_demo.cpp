#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
int main(int argc, char **argv){
  ros::init(argc, argv, "moveit_joint_pose_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface arm("xarm");
  moveit::planning_interface::MoveGroupInterface gripper("gripper");

  arm.setGoalJointTolerance(0.01);
  gripper.setGoalJointTolerance(0.01);
  arm.setMaxVelocityScalingFactor(0.8);
  gripper.setMaxVelocityScalingFactor(0.8);

  ROS_INFO("Moveing to joint-space goal: joint_positions");

  std::vector<double> arm_joint_positions = {-0.664, -0.775, 0.675, -1.241, -0.473, -1.281};
  arm.setJointValueTarget(arm_joint_positions);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("moveit_joint_pose_demo", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");
  if(success){
    arm.execute(plan);
  }
  ROS_INFO("Open gripper ...");
  std::vector<double> gripper_joint_positions = {0.65,0.65};
  gripper.setJointValueTarget(gripper_joint_positions);
  gripper.move();
  ROS_INFO("Close gripper ...");
  gripper.setNamedTarget("Close_gripper");
  gripper.move();
  ROS_INFO("Moving to pose: Home");
  arm.setNamedTarget("Home");
  arm.move();
  ros::shutdown();
  return 0;
}
