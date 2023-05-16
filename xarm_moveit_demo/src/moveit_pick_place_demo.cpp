#include <iostream>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
const std::string BASE_LINK = "base_link";
const std::string TABLE_ID = "table";
const std::string TARGET_ID = "target";
const std::vector<std::string> GRIPPER_JOINT_NAMES = {"gripper_1_joint", "gripper_2_joint"};
const std::vector<double> GRIPPER_OPEN = {0.65,0.65};
const std::vector<double> GRIPPER_GRASP = {0.1,0.1};

void makeGripperPosture(trajectory_msgs::JointTrajectory& posture, std::vector<double> positions)
{
  // 设置joint_names为gripper规划组的两个关节名
  posture.joint_names = GRIPPER_JOINT_NAMES;
  // 设置关节的位置
  posture.points.resize(1);
  posture.points[0].positions = positions;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void makeGrasps(std::vector<moveit_msgs::Grasp>& grasps){
  // 设置grasps里只包含一个元素
  grasps.resize(1);
  // 设置抓取的位姿grasp_pose
  grasps[0].grasp_pose.header.frame_id = BASE_LINK;
  grasps[0].grasp_pose.pose.position.x = 0.47;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.14;
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, 0);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  // 设置pre_grasp_approach,沿着X轴正向靠近抓取点,移动的最小距离为0.1m，期望距离为0.12米
  grasps[0].pre_grasp_approach.direction.header.frame_id = BASE_LINK;
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.1;
  grasps[0].pre_grasp_approach.desired_distance = 0.12;
  // 设置post_grasp_retreat,抓取物体后，沿着Z轴正向撤离，移动的距离最小为0.08米，期望距离为0.1米
  grasps[0].post_grasp_retreat.direction.header.frame_id = BASE_LINK;
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.08;
  grasps[0].post_grasp_retreat.desired_distance = 0.1;
  // 设置夹爪在抓取物品前的位姿为张开的状态
  makeGripperPosture(grasps[0].pre_grasp_posture, GRIPPER_OPEN);
  // 设置夹爪用于抓取对象时的位置
  makeGripperPosture(grasps[0].grasp_posture, GRIPPER_GRASP);
}

void makePlaces(std::vector<moveit_msgs::PlaceLocation> &place_locations, geometry_msgs::PoseStamped init_pose){
  // 创建moveit_msgs/PlaceLocation消息的对象place
  moveit_msgs::PlaceLocation place;
  // 设置放置位姿
  place.place_pose = init_pose;
  // 设置靠近放置点的方向、最小移动距离和期望距离，这里设置为沿着Z轴向下移动0.1米
  place.pre_place_approach.direction.header.frame_id = BASE_LINK;
  place.pre_place_approach.direction.vector.z = -1.0;
  place.pre_place_approach.min_distance = 0.08;
  place.pre_place_approach.desired_distance = 0.1;
  // 设置放置完成后机械臂的撤离方向、移动最小距离和期望距离，这里设置为沿着Z轴向上移动0.15米
  place.post_place_retreat.direction.header.frame_id = BASE_LINK;
  place.post_place_retreat.direction.vector.z = 1.0;
  place.post_place_retreat.min_distance = 0.12;
  place.post_place_retreat.desired_distance = 0.15;
  //  可尝试的x位置偏移量
  std::vector<double> x_vals = {0, 0.005, 0.01, -0.005, -0.01};
  // 在放置位置附近生成其他可放置位置，并添加到放置位姿列表place_locations
  for (auto x:x_vals) {
    place.place_pose.pose.position.x = init_pose.pose.position.x + x;
    place_locations.push_back(place);
  }
}


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  collision_objects[0].id = TABLE_ID;
  collision_objects[0].header.frame_id = BASE_LINK;

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.0;
  collision_objects[0].primitives[0].dimensions[1] = 1.2;
  collision_objects[0].primitives[0].dimensions[2] = 0.01;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = -0.005;
  collision_objects[0].operation = collision_objects[0].ADD;

  collision_objects[1].id = TARGET_ID;
  collision_objects[1].header.frame_id = BASE_LINK;

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.05;
  collision_objects[1].primitives[0].dimensions[1] = 0.05;
  collision_objects[1].primitives[0].dimensions[2] = 0.22;

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.47;
  collision_objects[1].primitive_poses[0].position.y = 0.0;
  collision_objects[1].primitive_poses[0].position.z = 0.11;
  collision_objects[1].primitive_poses[0].orientation.w = 1;

  collision_objects[1].operation = collision_objects[1].ADD;
  planning_scene_interface.addCollisionObjects(collision_objects);
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_pick_place_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("Pick and Place demo is ready.");

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface xarm_group("xarm");
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
  xarm_group.setMaxVelocityScalingFactor(0.8);
  gripper_group.setMaxVelocityScalingFactor(0.8);
  // 添加障碍物和目标物体
  addCollisionObjects(planning_scene_interface);
  ros::WallDuration(1.0).sleep();
  // 设置桌子table为抓取和放置操作的支撑面，使MoveIt!忽略物体放到桌子上时产生的碰撞警告
  xarm_group.setSupportSurfaceName(TABLE_ID);
  // 生成grasp抓取向量
  std::vector<moveit_msgs::Grasp> grasps;
  makeGrasps(grasps);
  // 尝试进行抓取
  ROS_INFO("Try to pick up the box. ");
  moveit::planning_interface::MoveItErrorCode result;
  int max_pick_attempts = 5;
  int n_attempts = 0;
  while(result != moveit::planning_interface::MoveItErrorCode::SUCCESS && n_attempts < max_pick_attempts ){
    n_attempts++;
    result = xarm_group.pick(TARGET_ID, grasps);
    ros::WallDuration(0.2).sleep();
  }

  ros::WallDuration(1.0).sleep();
  // 设置一个放置目标位姿
  geometry_msgs::PoseStamped place_pose;
  place_pose.header.frame_id = BASE_LINK;
  place_pose.pose.position.x = 0.32;
  place_pose.pose.position.y = -0.32;
  place_pose.pose.position.z = 0.22 / 2.0;
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, -M_PI / 4);
  place_pose.pose.orientation = tf2::toMsg(orientation);
  // 生成一系列放置位姿
  std::vector<moveit_msgs::PlaceLocation> place_locations;
  makePlaces(place_locations, place_pose);
  // 如果抓取成功，尝试物品放置操作
  if(result == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    ROS_INFO("Picked up successfully. Try to place the box.");
    int max_place_attempts = 5;
    n_attempts = 0;
    result = 0;
    while(result != moveit::planning_interface::MoveItErrorCode::SUCCESS && n_attempts < max_place_attempts ){
      n_attempts++;
      result = xarm_group.place(TARGET_ID, place_locations);
      ros::WallDuration(0.2).sleep();
    }
  }
  // 闭合手爪
  ROS_INFO("Close gripper ...");
  gripper_group.setNamedTarget("Close_gripper");
  gripper_group.move();
  // 回到初始位置
  ROS_INFO("Moving to pose: Home");
  xarm_group.setNamedTarget("Home");
  xarm_group.move();
  // 删除规划场景里的桌面和目标物体
  std::vector<std::string> object_ids = {TABLE_ID, TARGET_ID};
  planning_scene_interface.removeCollisionObjects(object_ids);
  ros::WallDuration(1.0).sleep();
  ros::shutdown();
  return 0;
}
