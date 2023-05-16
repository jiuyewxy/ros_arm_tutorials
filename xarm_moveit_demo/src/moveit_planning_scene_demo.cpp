#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
// 判断是否已成功添加物体到规划场景，或是否成功把物体附着到机械臂上
bool waitForStateUpdate(std::string obstacle_name, moveit::planning_interface::PlanningSceneInterface& scene, bool obstacle_is_known=false, bool object_is_attached=false, double  timeout=4){
  ros::Time start_time = ros::Time::now();
  ros::Time seconds = ros::Time::now();
  bool is_attached, is_known;
  while(seconds - start_time <ros::Duration(timeout) && !ros::isShuttingDown()){
    std::vector<std::string> attached_object_ids;
    attached_object_ids.push_back(obstacle_name);
    // 获取由指定ID列表标记的附着对象物体
    auto attached_objects = scene.getAttachedObjects(attached_object_ids);
    is_attached = attached_objects.size();
    // 获取规划场景中已有的所有物体的ID
    std::vector<std::string> world_object_names = scene.getKnownObjectNames();
    if(std::find(world_object_names.begin(),world_object_names.end(), obstacle_name) != world_object_names.end()){
      is_known =true;
    }
    else {
      is_known =false;
    }
    if(object_is_attached == is_attached && obstacle_is_known == is_known)
      return true;
    usleep(100000);
    seconds = ros::Time::now();
  }
  return false;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "moveit_planning_scene_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface arm("xarm");
  moveit::planning_interface::MoveGroupInterface gripper("gripper");
  // 创建PlanningSceneInterface对象scene用来对规划场景进行操作
  moveit::planning_interface::PlanningSceneInterface scene;
  // 设置一个比例因子以选择性地降低最大关节速度限制，可取值为(0,1]
  arm.setMaxVelocityScalingFactor(0.8);
  gripper.setMaxVelocityScalingFactor(0.8);
  // 机械臂回到初始位置
  ROS_INFO("Moving to pose: Home");
  arm.setNamedTarget("Home");
  arm.move();
  // 设置桌面对象的形状和位姿
  moveit_msgs::CollisionObject table_object;
  table_object.header.frame_id = "base_link";
  table_object.id = "table";
  shape_msgs::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions.resize(3);
  table_primitive.dimensions[0] = 1.0;
  table_primitive.dimensions[1] = 1.2;
  table_primitive.dimensions[2] = 0.01;
  geometry_msgs::Pose table_pose;
  table_pose.position.x = 0;
  table_pose.position.y = 0;
  table_pose.position.z = -table_primitive.dimensions[2]/2.0;
  table_pose.orientation.w = 1.0;
  table_object.primitives.push_back(table_primitive);
  table_object.primitive_poses.push_back(table_pose);
  table_object.operation = table_object.ADD;

  // 设置长方体对象的形状和位姿
  moveit_msgs::CollisionObject box_object;
  box_object.header.frame_id = "base_link";
  box_object.id = "box";
  shape_msgs::SolidPrimitive box_primitive;
  box_primitive.type = box_primitive.BOX;
  box_primitive.dimensions.resize(3);
  box_primitive.dimensions[0] = 0.25;
  box_primitive.dimensions[1] = 0.3;
  box_primitive.dimensions[2] = 0.04;
  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0;
  box_pose.position.z = box_primitive.dimensions[2]/2.0 + 0.37;
  box_pose.orientation.w = 1.0;
  box_object.primitives.push_back(box_primitive);
  box_object.primitive_poses.push_back(box_pose);
  box_object.operation = box_object.ADD;

  // 设置圆柱体对象的形状和位姿
  moveit_msgs::CollisionObject cylinder_object;
  cylinder_object.header.frame_id = "base_link";
  cylinder_object.id = "cylinder";
  shape_msgs::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = cylinder_primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[0] = 0.12;
  cylinder_primitive.dimensions[1] = 0.015;
  geometry_msgs::Pose cylinder_pose;
  cylinder_pose.position.x = 0.4;
  cylinder_pose.position.y = 0.0;
  cylinder_pose.position.z = cylinder_primitive.dimensions[0]/2.0 + box_primitive.dimensions[2]/2.0 + 0.37;
  cylinder_pose.orientation.w = 1.0;
  cylinder_object.primitives.push_back(cylinder_primitive);
  cylinder_object.primitive_poses.push_back(cylinder_pose);
  cylinder_object.operation = cylinder_object.ADD;
  // 将桌面、长方体和圆柱体添加到collision_objects
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(table_object);
  collision_objects.push_back(box_object);
  collision_objects.push_back(cylinder_object);
  // 将桌面、长方体和圆柱体添加到规划场景中
  scene.addCollisionObjects(collision_objects);
  scene.applyCollisionObjects(collision_objects);

  // 判断桌面是否已被添加到规划场景中
  if(waitForStateUpdate(table_object.id,scene,true)){
    ROS_INFO("The table has been successfully added.");
  }
  else{
    ROS_INFO("Failed to add the table.");
  }
  // 判断长方体是否已被添加到规划场景中
  if(waitForStateUpdate(box_object.id,scene,true)){
    ROS_INFO("The Box has been successfully added.");
  }
  else{
    ROS_INFO("Failed to add the Box.");
  }
  // 判断圆柱体是否已被添加到规划场景中
  if(waitForStateUpdate(cylinder_object.id,scene,true)){
    ROS_INFO("The Cylinder has been successfully added.");
  }
  else{
    ROS_INFO("Failed to add the Cylinder.");
  }

  ROS_INFO("Open gripper, move the arm and close gripper...");
  // 张开手爪
  std::vector<double> gripper_joint_positions = {0.65,0.65};
  gripper.setJointValueTarget(gripper_joint_positions);
  gripper.move();
  // 设置xarm规划组的目标位姿为圆柱体中心附近，让机械臂运动到该目标
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x = 0.41;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = cylinder_pose.position.z;
  target_pose.pose.orientation.w = 1;
  arm.setStartStateToCurrentState();
  arm.setPoseTarget(target_pose);
  arm.move();
  // 稍微闭合手爪
  gripper_joint_positions = {0.2,0.2};
  gripper.setJointValueTarget(gripper_joint_positions);
  gripper.move();
  ROS_INFO("Attach the Cylinder to the XBot-Arm robot ...");
  // 把cylinder附着到机械臂末端执行器上
  arm.attachObject(cylinder_object.id);
  // 判断圆柱体是否成功附着到了机械臂上
  if(waitForStateUpdate(cylinder_object.id,scene,false, true)){
    ROS_INFO("The Cylinder has been successfully attached.");
  }
  else{
    ROS_INFO("Failed to attach the Cylinder.");
  }
  ROS_INFO("Move the arm to a position 25 cm down ...");
  // 让机械臂运动到长方体下方的位置
  target_pose.pose.position.z -= 0.25;
  arm.setStartStateToCurrentState();
  arm.setPoseTarget(target_pose);
  arm.move();
  ROS_INFO("Detach the Cylinder from the XBot-Arm robot");
  // 将圆柱体与机械臂分离
  arm.detachObject(cylinder_object.id);
  sleep(2);
  ROS_INFO("Remove the objects from planning scene");
  // 删除规划场景里的物体对象
  std::vector<std::string> object_ids;
  object_ids.push_back(table_object.id);
  object_ids.push_back(box_object.id);
  object_ids.push_back(cylinder_object.id);
  scene.removeCollisionObjects(object_ids);
  sleep(1);
  // 机械臂回到初始位置
  ROS_INFO("Moving to pose: Home");
  arm.setNamedTarget("Home");
  arm.move();

  ros::shutdown();
  return 0;
}
