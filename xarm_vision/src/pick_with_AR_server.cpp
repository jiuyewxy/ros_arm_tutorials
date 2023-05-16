#include "pick_with_AR_server.h"
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ARTrackAndPick::ARTrackAndPick(ros::NodeHandle &nh):nh_(nh),table_id_("table"){
  // 创建话题/ar_pose_marker的订阅端
  ar_marker_sub_ = nh_.subscribe(std::string("/ar_pose_marker"), 100, &ARTrackAndPick::arMarkerCallback, this);
  // 创建服务/xarm_vision_pickup的服务端
  pick_place_srv_ = nh_.advertiseService(std::string("/xarm_vision_pickup"), &ARTrackAndPick::callPickPlace, this);
  ROS_INFO("Pick and Place demo is ready. You can call the service /xarm_vision_pickup to test... ");

}

// 话题/ar_pose_marker的回调函数
void ARTrackAndPick::arMarkerCallback(
  const ar_track_alvar_msgs::AlvarMarkers msg) {
  ar_markers_ = msg;
}

// /xarm_vision_pickup服务处理函数
bool ARTrackAndPick::callPickPlace(xarm_vision::CallPickPlaceDemo::Request &req,
                                   xarm_vision::CallPickPlaceDemo::Response &res){
  if(ar_markers_.markers.size() == 0){
    ROS_INFO("No target found!!!");
    res.success = false;
    return true;
  }
  // 把桌子添加到规划场景中
  addDeskFloor();
  sleep(1);
  moveit::planning_interface::MoveGroupInterface xarm("xarm");
  moveit::planning_interface::MoveGroupInterface gripper("gripper");
  xarm.allowReplanning(true);
  // 设置桌子table为抓取和放置操作的支撑面，使MoveIt!忽略物体放到桌子上时产生的碰撞警告
  xarm.setSupportSurfaceName(table_id_);
  // 在规划场景中添加所有的目标物体，并记录每个目标的位姿
  std::vector<std::string> target_ids;
  std::vector<geometry_msgs::PoseStamped> target_poses;
  std::vector<double> yaw_offset;
  for (auto tag : ar_markers_.markers) {
    tag.pose.header.frame_id = "base_link";
    target_ids.push_back(std::to_string(tag.id));
    target_poses.push_back(tag.pose);
    // tag.pose.pose.position.z = tag.pose.pose.position.z/2.0;
    // 获取AR标签的姿态YAW表示
    double tag_yaw = tf::getYaw(tag.pose.pose.orientation);
    // yaw_offset列表用于保存每个目标方块的偏航角与抓取时的gripper_centor_link的偏航角yaw的差值
    double yaw = atan2(tag.pose.pose.position.y,tag.pose.pose.position.x);
    yaw_offset.push_back(tag_yaw - yaw);
    // 设置方块的姿态与欧拉角的Yaw偏航角一致，Roll和Pitch为零
    tf2::Quaternion q;
    q.setRPY(0,0,tag_yaw);
    tag.pose.pose.orientation.x = q.x();
    tag.pose.pose.orientation.y = q.y();
    tag.pose.pose.orientation.z = q.z();
    tag.pose.pose.orientation.w = q.w();
    // 在规划场景中添加目标方块
    addBox(std::to_string(tag.id), tag.pose.pose);
    sleep(1);
  }
  // 设置一个放置目标位姿place_pose
  geometry_msgs::PoseStamped place_pose;
  place_pose.header.frame_id = "base_link";
  place_pose.pose.position.x = 0.25;
  place_pose.pose.position.y = 0.25;
  place_pose.pose.position.z = 0.07 / 2.0;

  // 抓取目标并放置到指定位置
  for (int i=0; i<target_ids.size();i++) {
    if(pickupCube(xarm, target_ids[i], target_poses[i])){
      ROS_INFO("Pickup  successfully. ");
      tf2::Quaternion q;
      q.setRPY(0,0,3.1415926/4.0 + yaw_offset[i]);
      place_pose.pose.orientation.x = q.x();
      place_pose.pose.orientation.y = q.y();
      place_pose.pose.orientation.z = q.z();
      place_pose.pose.orientation.w = q.w();
      if(placeCube(xarm,target_ids[i],place_pose)){
        place_pose.pose.position.z += 0.07;
      }else{
        ROS_ERROR("Place Failed !!!");
        res.success = false;
        return true;
      }
    }else{
      ROS_ERROR("Pickup Failed !!!");
      res.success = false;
      return true;
    }
  }
  gripper.setNamedTarget("Close_gripper");
  gripper.move();
  // 回到初始位置
  ROS_INFO("Moving to pose: Home");
  xarm.setNamedTarget("Home");
  xarm.move();
  // 删除规划场景里的桌面和目标物体
  planning_scene_interface_.removeCollisionObjects(target_ids);
  ros::WallDuration(1.0).sleep();
  res.success = true;
  return true;
}

// 在规划场景中添加目标方块
void ARTrackAndPick::addBox(std::string target_id, geometry_msgs::Pose pose) {
  moveit_msgs::CollisionObject cube_object;
  cube_object.header.frame_id = "base_link";
  cube_object.id = target_id;
  geometry_msgs::Pose object_pose;
  object_pose.orientation = pose.orientation;
  object_pose.position.x = pose.position.x;
  object_pose.position.y = pose.position.y;
  object_pose.position.z = 0.035;
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.07;
  primitive.dimensions[1] = 0.07;
  primitive.dimensions[2] = 0.07;
  cube_object.primitive_poses.clear();
  cube_object.primitives.clear();
  cube_object.primitives.push_back(primitive);
  cube_object.primitive_poses.push_back(object_pose);
  cube_object.operation = cube_object.ADD;
  collision_objects_.push_back(cube_object);
  planning_scene_interface_.addCollisionObjects(collision_objects_);
  planning_scene_interface_.applyCollisionObjects(collision_objects_);
}


// 添加桌面到规划场景中
void ARTrackAndPick::addDeskFloor() {
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = table_id_;
  geometry_msgs::Pose object_pose;
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.0;
  primitive.dimensions[1] = 1.2;
  primitive.dimensions[2] = 0.01;
  object_pose.orientation.w = 1.0;
  object_pose.position.x = 0.0;
  object_pose.position.y = 0;
  object_pose.position.z = -primitive.dimensions[2]/2.0;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(object_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects_.push_back(collision_object);
  planning_scene_interface_.addCollisionObjects(collision_objects_);
  planning_scene_interface_.applyCollisionObjects(collision_objects_);
}

// 设置抓取姿态并抓取目标方块
bool ARTrackAndPick::pickupCube(
    moveit::planning_interface::MoveGroupInterface &arm_group,
    std::string target_id, geometry_msgs::PoseStamped target_pose) {
  // 设置grasps里只包含一个元素
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  // 设置抓取的位姿grasp_pose
  grasps[0].grasp_pose.header.frame_id = "base_link";
  double yaw;
  yaw = atan2(target_pose.pose.position.y, target_pose.pose.position.x);
  tf2::Quaternion orientation;
  orientation.setRPY(0, 3.1415926 / 2.0, yaw);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = target_pose.pose.position.x;
  grasps[0].grasp_pose.pose.position.y = target_pose.pose.position.y;
  grasps[0].grasp_pose.pose.position.z = 0.08;
  // 设置pre_grasp_approach,沿着z轴负向靠近抓取点,移动的最小距离为0.06m，期望距离为0.1米
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.06;
  grasps[0].pre_grasp_approach.desired_distance = 0.1;
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
  // 设置post_grasp_retreat,抓取物体后，沿着Z轴正向撤离，移动的距离最小为0.086米，期望距离为0.1米
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.06;
  grasps[0].post_grasp_retreat.desired_distance = 0.1;

  // 设置夹爪在抓取物品前的位姿为张开的状态
  grasps[0].pre_grasp_posture.joint_names.resize(2);
  grasps[0].pre_grasp_posture.joint_names[0] = "gripper_1_joint";
  grasps[0].pre_grasp_posture.joint_names[1] = "gripper_2_joint";
  grasps[0].pre_grasp_posture.points.resize(1);
  grasps[0].pre_grasp_posture.points[0].positions.resize(2);
  grasps[0].pre_grasp_posture.points[0].positions[0] = 0.68;
  grasps[0].pre_grasp_posture.points[0].positions[1] = 0.68;
  grasps[0].pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

  // 设置夹爪用于抓取对象时的位置
  grasps[0].grasp_posture.joint_names.resize(2);
  grasps[0].grasp_posture.joint_names[0] = "gripper_1_joint";
  grasps[0].grasp_posture.joint_names[1] = "gripper_2_joint";
  grasps[0].grasp_posture.points.resize(1);
  grasps[0].grasp_posture.points[0].positions.resize(2);
  grasps[0].grasp_posture.points[0].positions[0] = 0.25;
  grasps[0].grasp_posture.points[0].positions[1] = 0.25;
  grasps[0].grasp_posture.points[0].time_from_start = ros::Duration(5);

  arm_group.setSupportSurfaceName(table_id_);
  arm_group.setPlanningTime(5);
  ROS_INFO("Try to pickup the object ......");
  int max_pick_attempts = 5;
  int n_attempts = 0;
  moveit::planning_interface::MoveItErrorCode result;
  // 尝试进行抓取
  while(result != moveit::planning_interface::MoveItErrorCode::SUCCESS && n_attempts < max_pick_attempts ){
    n_attempts++;
    result = arm_group.pick(target_id, grasps);
    ros::WallDuration(0.2).sleep();
  }

  if(result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    return true;
  }
  else {
    return false;
  }
}

bool ARTrackAndPick::placeCube(
    moveit::planning_interface::MoveGroupInterface &arm_group,
    std::string target_id, geometry_msgs::PoseStamped place_pose) {
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);
  // 设置放置位姿
  place_location[0].place_pose.header.frame_id = "base_link";
  place_location[0].place_pose.pose = place_pose.pose;

  // 设置靠近放置点的方向、最小移动距离和期望距离
  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.05;
  place_location[0].pre_place_approach.desired_distance = 0.08;

  // 设置放置完成后机械臂的撤离方向、移动最小距离和期望距离
  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.06;
  place_location[0].post_place_retreat.desired_distance = 0.08;

  // Setting posture of eef after placing object
  place_location[0].post_place_posture.joint_names.resize(2);
  place_location[0].post_place_posture.joint_names[0] = "gripper_1_joint";
  place_location[0].post_place_posture.joint_names[1] = "gripper_2_joint";

  place_location[0].post_place_posture.points.resize(1);
  place_location[0].post_place_posture.points[0].positions.resize(2);
  place_location[0].post_place_posture.points[0].positions[0] = 0.68;
  place_location[0].post_place_posture.points[0].positions[1] = 0.68;
  place_location[0].post_place_posture.points[0].time_from_start =
  ros::Duration(0.5);

  arm_group.setSupportSurfaceName(table_id_);
  // 尝试物品放置操作
  int max_place_attempts = 5;
  int n_attempts = 0;
  moveit::planning_interface::MoveItErrorCode result;
  ROS_INFO("Try to place ...... ");

  while (n_attempts < max_place_attempts) {
    ++n_attempts;
    result = arm_group.place(target_id, place_location);
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      return true;
  }
  return false;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pick_with_AR_server");
  ros::NodeHandle nh("");
  ARTrackAndPick ARTrackAndPick(nh);
  ros::AsyncSpinner spinner(6);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}














