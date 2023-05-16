#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "advance_demo/PickupPlaceAction.h"

// doneCB在目标执行完时被调用一次
void doneCB(const actionlib::SimpleClientGoalState& state,
            const advance_demo::PickupPlaceResultConstPtr& result){
  if(state.toString() == "PREEMPTED"){
    ROS_INFO("Program interrupted before completion");
  }
  if(result->success){
    ROS_INFO("Pickup and place the object successfully!");
  }
}

// activeCB在目标状态转换为ACTIVE时被调用
void activeCB(){
  ROS_INFO("Goal just went active");
}

// feedbackCB在每次接收到服务端发送的feedback时被调用
void feedbackCB(const advance_demo::PickupPlaceFeedbackConstPtr& feedback){
  ROS_INFO("Task completed %i%%", feedback->percent_complete);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "action_client");
  actionlib::SimpleActionClient<advance_demo::PickupPlaceAction> ac("pickup_place", true);
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();
  ROS_INFO("Action server started, sending goal.");
  // 创建一个PickupPlaceGoal的对象,并对goal的成员进行赋值
  advance_demo::PickupPlaceGoal goal;
  goal.target_name = "box";
  goal.target_pose.position.x = 0.35;
  goal.target_pose.position.y = -0.35;
  goal.target_pose.position.z = 0.1;
  goal.target_pose.orientation.w = 1;
  // 向action的服务端发送目标goal,并设置回调函数
  ac.sendGoal(goal, &doneCB, &activeCB, &feedbackCB);
  // 阻塞,直到这个目标完成
  ac.waitForResult();
  return 0;
}
