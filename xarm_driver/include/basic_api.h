#ifndef BASIC_API_H
#define BASIC_API_H
#include <ros/ros.h>
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <xarm_driver/CommandPose.h>
#include <xarm_driver/CommandJoint.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
static const std::vector<double> OPEN_GRIPPER_POSE = {0.65,0.65};
static const std::vector<double> CLOSE_GRIPPER_POSE = {0.0,0.0};
static const std::vector<double> HOME_POSITION = {0, 0, 0, 0, 0, 0};

class ArmBasicAPI{
public:
  ArmBasicAPI(ros::NodeHandle nodehandle){
    control_nodehandle =nodehandle;
  }
  ~ArmBasicAPI(){};
  void basicInit();

private:
  int xarm_mode_;
  ros::NodeHandle control_nodehandle;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  void setCollisionObject();

  void advertiseServices();
  void subscribeTopicsSingleARM();
  void subscribeTopicsDoubleARM();
  /*********************
   ** Ros ServiceServer
   **********************/
  ros::ServiceServer command_pose_srv;
  ros::ServiceServer command_joint_srv;


  /*********************
   ** Ros Publishers
   **********************/

  /*********************
   ** Ros Subscribers
   **********************/
  ros::Subscriber arm_lift_up_sub_, arm_put_down_sub_;
  ros::Subscriber left_arm_lift_up_sub_, left_arm_put_down_sub_;
  ros::Subscriber right_arm_lift_up_sub_, right_arm_put_down_sub_;

  void armCommandUPCB(const std_msgs::Empty msg);
  void armCommandDownCB(const std_msgs::Empty msg);

  void armLeftCommandUPCB(const std_msgs::Empty msg);
  void armLeftCommandDownCB(const std_msgs::Empty msg);

  void armRightCommandUPCB(const std_msgs::Empty msg);
  void armRightCommandDownCB(const std_msgs::Empty msg);

  void commandJointCB(const sensor_msgs::JointState );

  bool commandPoseCB(xarm_driver::CommandPose::Request &req,xarm_driver::CommandPose::Response &res);
  bool commandJointCB(xarm_driver::CommandJoint::Request &req,xarm_driver::CommandJoint::Response &res);

  bool openGripper( moveit::planning_interface::MoveGroupInterface & gripper_group);
  bool closeGripper( moveit::planning_interface::MoveGroupInterface & gripper_group);

};
#endif  // XARM_ROS_H
