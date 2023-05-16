#ifndef PICK_WITH_AR_SERVER_H
#define PICK_WITH_AR_SERVER_H
#include <ros/ros.h>
#include <iostream>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "xarm_vision/CallPickPlaceDemo.h"

class ARTrackAndPick
{
public:
  ARTrackAndPick(ros::NodeHandle &nh);

private:
  ros::NodeHandle nh_;
  ros::Subscriber ar_marker_sub_;
  ros::ServiceServer pick_place_srv_;
  ar_track_alvar_msgs::AlvarMarkers ar_markers_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::string table_id_;
  std::vector<moveit_msgs::CollisionObject> collision_objects_;


  void arMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers msg);
  bool callPickPlace(xarm_vision::CallPickPlaceDemo::Request &req,
                        xarm_vision::CallPickPlaceDemo::Response &res);
  void addDeskFloor();
  void addBox(std::string target_id, geometry_msgs::Pose pose);

  bool pickupCube(moveit::planning_interface::MoveGroupInterface &arm_group,
                  std::string target_id, geometry_msgs::PoseStamped target_pose);

  bool placeCube(moveit::planning_interface::MoveGroupInterface &arm_group,
                 std::string target_id, geometry_msgs::PoseStamped place_pose);

};



#endif
