#include "basic_api.h"
#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

std::string group_name = "xarm";

// 初始化发布以及订阅的话题
void ArmBasicAPI::basicInit(){
  //setCollisionObject();
  if (!control_nodehandle.getParam("/arm_api/xarm_mode", xarm_mode_)) {
    ROS_ERROR(
        "Xarm : no arm xarm_mode_ given on the parameter server (e.g. "
        "/dev/xarm_mode).");
    return;
  }
  if(xarm_mode_ == 1){
    subscribeTopicsSingleARM();
  }  else if(xarm_mode_ ==2){
    subscribeTopicsDoubleARM();
  }

  //advertiseServices();
}


/** @brief 张开手爪
 *  @param gripper_group:手爪的movegroup
 *  @return true if success
 */
bool ArmBasicAPI::openGripper( moveit::planning_interface::MoveGroupInterface & gripper_group){
  sensor_msgs::JointState joint_state;
  gripper_group.setStartStateToCurrentState();

  gripper_group.setJointValueTarget(OPEN_GRIPPER_POSE);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (gripper_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
    return false;
  }
  ROS_INFO("--- Open gripper! ---");
  gripper_group.move();
  return true;

}

/** @brief 闭合手爪
 *  @param gripper_group:手爪的movegroup
 *  @return true if success
 */
bool ArmBasicAPI::closeGripper( moveit::planning_interface::MoveGroupInterface & gripper_group){
  sensor_msgs::JointState joint_state;
  gripper_group.setStartStateToCurrentState();

  gripper_group.setJointValueTarget(CLOSE_GRIPPER_POSE);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (gripper_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
    return false;
  }
  ROS_INFO("--- Close gripper! ---");
  gripper_group.move();
  return true;

}

// 把桌子作为障碍物添加到plan_scene三
void ArmBasicAPI::setCollisionObject(){
  namespace rvt = rviz_visual_tools;
  // 连接机械臂实例组
  moveit::planning_interface::MoveGroupInterface group("xarm");
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();
  // The id of the object is used to identify it.
  collision_object.id = "desk_floor";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.5;
  primitive.dimensions[1] = 1.5;
  primitive.dimensions[2] = 0.01;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = 0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

}

void ArmBasicAPI::advertiseServices(){
  command_pose_srv = control_nodehandle.advertiseService(std::string("xbot_arm/command/pose"),&ArmBasicAPI::commandPoseCB,this);
  command_joint_srv = control_nodehandle.advertiseService(std::string("xbot_arm/command/joint"),&ArmBasicAPI::commandJointCB,this);

}

// 订阅的话题。
void ArmBasicAPI::subscribeTopicsSingleARM()
{
  arm_lift_up_sub_ = control_nodehandle.subscribe(std::string("/arm/commands/lift_up"), 10,
                                        &ArmBasicAPI::armCommandUPCB, this);
  arm_put_down_sub_ = control_nodehandle.subscribe(std::string("/arm/commands/put_down"), 10,
                                        &ArmBasicAPI::armCommandDownCB, this);
}

void ArmBasicAPI::subscribeTopicsDoubleARM()
{

  left_arm_lift_up_sub_ = control_nodehandle.subscribe(std::string("/left_arm/commands/lift_up"), 10,
                                        &ArmBasicAPI::armLeftCommandUPCB, this);
  left_arm_put_down_sub_ = control_nodehandle.subscribe(std::string("/left_arm/commands/put_down"), 10,
                                        &ArmBasicAPI::armLeftCommandDownCB, this);

  right_arm_lift_up_sub_ = control_nodehandle.subscribe(std::string("/right_arm/commands/lift_up"), 10,
                                        &ArmBasicAPI::armRightCommandUPCB, this);
  right_arm_put_down_sub_ = control_nodehandle.subscribe(std::string("/right_arm/commands/put_down"), 10,
                                        &ArmBasicAPI::armRightCommandDownCB, this);
}

void ArmBasicAPI::armCommandUPCB(const std_msgs::Empty msg){
  moveit::planning_interface::MoveGroupInterface arm_group("xarm");
  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  ROS_INFO_STREAM("Current pose is : \n"<<current_pose.position.x<<"\n"<<current_pose.position.y<<"\n"<<current_pose.position.z<<std::endl);
  // 设置start_pose为当前的位置。
  arm_group.setStartStateToCurrentState();
  geometry_msgs::Pose target_pose;
  tf2::Quaternion orientation;

  target_pose.position.x = 0.55;
  target_pose.position.y = 0;
  target_pose.position.z = 0.85;
  //double yaw = atan2(target_pose.position.y,target_pose.position.x);
  orientation.setRPY(0.0, 0.0, 0);
  target_pose.orientation = tf2::toMsg(orientation);
  arm_group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success){
    ROS_INFO("--- Plan trajectory Success --- ");
    arm_group.move();
  }else {
    ROS_ERROR("--- Plan trajectory failed --- ");
  }
}

void ArmBasicAPI::armCommandDownCB(const std_msgs::Empty msg){
  moveit::planning_interface::MoveGroupInterface arm_group("xarm");
  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  ROS_INFO_STREAM("Current pose is : \n"<<current_pose.position.x<<"\n"<<current_pose.position.y<<"\n"<<current_pose.position.z<<std::endl);
  // 设置start_pose为当前的位置。
  arm_group.setStartStateToCurrentState();
  arm_group.setJointValueTarget(HOME_POSITION);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success){
    ROS_INFO("--- Plan trajectory Success --- ");
    arm_group.move();
  }else {
    ROS_ERROR("--- Plan trajectory failed --- ");
  }
}

void ArmBasicAPI::armLeftCommandUPCB(const std_msgs::Empty msg){
  moveit::planning_interface::MoveGroupInterface arm_group("left_arm");
  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  ROS_INFO_STREAM("Current pose is : \n"<<current_pose.position.x<<"\n"<<current_pose.position.y<<"\n"<<current_pose.position.z<<std::endl);
  // 设置start_pose为当前的位置。
  arm_group.setStartStateToCurrentState();
  geometry_msgs::Pose target_pose;
  tf2::Quaternion orientation;

  target_pose.position.x = 0.45;
  target_pose.position.y = 0.195;
  target_pose.position.z = 0.85;
  //double yaw = atan2(target_pose.position.y,target_pose.position.x);
  orientation.setRPY(0.0, 0.0, 0);
  target_pose.orientation = tf2::toMsg(orientation);
  arm_group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success){
    ROS_INFO("--- Plan trajectory Success --- ");
    arm_group.move();
  }else {
    ROS_ERROR("--- Plan trajectory failed --- ");
  }
}

void ArmBasicAPI::armLeftCommandDownCB(const std_msgs::Empty msg){
  moveit::planning_interface::MoveGroupInterface arm_group("left_arm");
  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  ROS_INFO_STREAM("Current pose is : \n"<<current_pose.position.x<<"\n"<<current_pose.position.y<<"\n"<<current_pose.position.z<<std::endl);
  // 设置start_pose为当前的位置。
  arm_group.setStartStateToCurrentState();
  arm_group.setJointValueTarget(HOME_POSITION);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success){
    ROS_INFO("--- Plan trajectory Success --- ");
    arm_group.move();
  }else {
    ROS_ERROR("--- Plan trajectory failed --- ");
  }
}

void ArmBasicAPI::armRightCommandUPCB(const std_msgs::Empty msg){
  moveit::planning_interface::MoveGroupInterface arm_group("right_arm");
  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  ROS_INFO_STREAM("Current pose is : \n"<<current_pose.position.x<<"\n"<<current_pose.position.y<<"\n"<<current_pose.position.z<<std::endl);
  // 设置start_pose为当前的位置。
  arm_group.setStartStateToCurrentState();
  geometry_msgs::Pose target_pose;
  tf2::Quaternion orientation;

  target_pose.position.x = 0.45;
  target_pose.position.y = -0.195;
  target_pose.position.z = 0.85;
  //double yaw = atan2(target_pose.position.y,target_pose.position.x);
  orientation.setRPY(0.0, 0.0, 0);
  target_pose.orientation = tf2::toMsg(orientation);
  arm_group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success){
    ROS_INFO("--- Plan trajectory Success --- ");
    arm_group.move();
  }else {
    ROS_ERROR("--- Plan trajectory failed --- ");
  }
}

void ArmBasicAPI::armRightCommandDownCB(const std_msgs::Empty msg){
  moveit::planning_interface::MoveGroupInterface arm_group("right_arm");
  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  ROS_INFO_STREAM("Current pose is : \n"<<current_pose.position.x<<"\n"<<current_pose.position.y<<"\n"<<current_pose.position.z<<std::endl);
  // 设置start_pose为当前的位置。
  arm_group.setStartStateToCurrentState();
  arm_group.setJointValueTarget(HOME_POSITION);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success){
    ROS_INFO("--- Plan trajectory Success --- ");
    arm_group.move();
  }else {
    ROS_ERROR("--- Plan trajectory failed --- ");
  }
}


// 话题xbot_arm/command/joint的回调函数：机械臂规划到达指定臂型的路径并执行此路径
void ArmBasicAPI::commandJointCB(const sensor_msgs::JointState){


}

// 话题xbot_arm/command/pose的回调函数：令机械臂规划到达指定空间位置的路径并执行此路径
bool ArmBasicAPI::commandPoseCB(xarm_driver::CommandPose::Request &req,xarm_driver::CommandPose::Response &res){
  moveit::planning_interface::MoveGroupInterface arm_group(group_name);
  std::string plan_frame = arm_group.getPlanningFrame();
  ROS_INFO_STREAM("Plan frame is : "<<plan_frame);
  // 查看当前的位置
  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  ROS_INFO_STREAM("Current pose is : \n"<<current_pose.position.x<<"\n"<<current_pose.position.y<<"\n"<<current_pose.position.z<<std::endl);
  // 设置start_pose为当前的位置。
  arm_group.setStartStateToCurrentState();
  geometry_msgs::PoseStamped target_pose = req.PoseStamped;
  arm_group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
    res.success = false;
    return false;
  }
  ROS_INFO("--- Plan trajectory Success and start moving --- ");
  arm_group.move();
//  arm_group.getMoveGroupClient().waitForResult();
//  if (arm_group.getMoveGroupClient().getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//    ROS_INFO("You have reached the goal!");
//  else
//    ROS_INFO("The base failed for some reason");
    res.success = true;
    return true;
}

bool ArmBasicAPI::commandJointCB(xarm_driver::CommandJoint::Request &req,xarm_driver::CommandJoint::Response &res){
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  std::string plan_frame = move_group.getPlanningFrame();
  ROS_INFO_STREAM("Plan frame is : "<<plan_frame);
  std::vector<double> current_joint_value = move_group.getCurrentJointValues();
  ROS_INFO("Current joint value is : ");
  for (int i = 0; i < current_joint_value.size() ; ++i) {
    ROS_INFO_STREAM(current_joint_value[i]);
  }
  // 设置start_pose为当前的位置。
  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(req.JointState);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xbot_arm/command/joint", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
    res.success = false;
    return false;
  }
  ROS_INFO("--- Plan trajectory Success and start moving --- ");
  move_group.move();
  res.success = true;
  return true;

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_api");
  ros::NodeHandle nh;
  ArmBasicAPI arm_api(nh);
  arm_api.basicInit();
  ros::AsyncSpinner spinner(6);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
