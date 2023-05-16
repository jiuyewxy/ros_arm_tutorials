#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "joint_states_pub");
  ros::NodeHandle nh;
  ROS_INFO("joint_states_pub node is Ready!");
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  sensor_msgs::JointState joint_state;
  joint_state.name = {"arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint" , "arm_5_joint","arm_6_joint", "gripper_1_joint", "gripper_2_joint"};
  joint_state.position = {0, 0, 0, 0, 0, 0, 0, 0};
  ros::Rate rate(20);
  while(ros::ok()){
    for (int i = 0; i < 100; ++i) {
      joint_state.header.stamp = ros::Time::now();
      joint_state.position[0] += 0.02;
      joint_state.position[3] -= 0.015;
      joint_state.position[6] += 0.0065;
      joint_state.position[7] += 0.0065;
      joint_state_pub.publish(joint_state);
      rate.sleep();
    }
    for (int i = 0; i < 100; ++i) {
      joint_state.header.stamp = ros::Time::now();
      joint_state.position[0] -= 0.02;
      joint_state.position[3] += 0.015;
      joint_state.position[6] -= 0.0065;
      joint_state.position[7] -= 0.0065;
      joint_state_pub.publish(joint_state);
      rate.sleep();
    }
  }
  return 0;
}
