/******************* LICENSE AND COPYRIGHT **********************/
/*
Software License Agreement (BSD License)

Copyright (c) 2019, rocwang @ DROID All rights reserved.
Email: yowlings@droid.ac.cn.
Github: https://github.com/yowlings

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of the DROID nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
/**************** LICENSE AND COPYRIGHT END ***********************/
#ifndef XARM_ROS_WRAPPER_H_
#define XARM_ROS_WRAPPER_H_
#include <actionlib/server/simple_action_server.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf2_ros/transform_listener.h>
#include <xarm_driver/MotorStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "xarm_driver/SingleJointControl.h"
#include <serial/serial.h>
#include "xarm_driver/JointLocation.h"
#include <ecl/devices.hpp>
#include <ecl/threads.hpp>
#include <xarm_driver/MotorStatus.h>
#include <xarm_driver/CallVersion.h>
#include "packet/motor_status.h"
#include "packet_handler/packet_finder.h"
#include "command.h"
#include <fstream>
#include <iostream>
namespace xarm{
static const std::vector<std::string> CONTROL_LEFT_ARM_JOINTS = { "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint",
                                                         "arm_left_4_joint",      "arm_left_5_joint",       "arm_left_6_joint" };
static const std::vector<std::string> CONTROL_RIGHT_ARM_JOINTS = { "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint",
                                                         "arm_right_4_joint",      "arm_right_5_joint",       "arm_right_6_joint" };

static const std::vector<std::string> CONTROL_LEFT_GRIP_JOINTS = {"gripper_left_1_joint", "gripper_left_2_joint"};
static const std::vector<std::string> CONTROL_RIGHT_GRIP_JOINTS = {"gripper_right_1_joint", "gripper_right_2_joint"};
static const std::vector<std::string> CONTROL_ARM_JOINTS = { "arm_1_joint", "arm_2_joint", "arm_3_joint",
                                                         "arm_4_joint",      "arm_5_joint",       "arm_6_joint" };
static const std::vector<std::string> CONTROL_GRIP_JOINTS = {"gripper_1_joint", "gripper_2_joint"};

static const std::vector<double> DEFAULT_ARM_SPEED = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};

static const std::vector<std::string> CONTROL_SINGLE_JOINTS = { "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint",
                                                         "arm_left_4_joint",      "arm_left_5_joint",       "arm_left_6_joint", "gripper_1_joint", "gripper_2_joint" };

// 串口数据解码程序，使用有限状态机原理
class PacketFinder : public PacketFinderBase {
 public:
  virtual ~PacketFinder() {}
  //重定义校验函数
  bool checkSum();
};


// xarm的ros封装
class XarmRosWrapper {
 public:
  XarmRosWrapper(ros::NodeHandle &nh, std::string serial_port,
                 int baud_rate);
  ~XarmRosWrapper();
  void halt();

 private:
  int xarm_mode_;  // 选择单臂or双臂模式
  // 定义action的服务端、目标控制句柄以及action的反馈
  typedef control_msgs::FollowJointTrajectoryAction Action;
  typedef control_msgs::FollowJointTrajectoryResult Result;
  typedef actionlib::ServerGoalHandle<Action> GoalHandle;
  typedef actionlib::ActionServer<Action> ActionServer;

  ros::NodeHandle nh_;
  // Topics
  ros::Publisher motor_status_pub_, joint_state_pub_, location_pub_, left_location_pub_, right_location_pub_;
  ros::Subscriber gripper_sub_, single_joint_control_sub_, reset_sub_, left_gripper_sub_, right_gripper_sub_,check_test_;
  ros::Subscriber arm_check_,left_arm_check_,right_arm_check_;
  ros::ServiceServer call_version_;
  void advertiseTopics();
  void subscribeTopics();
  bool callVersionSrv(xarm_driver::CallVersion::Request &req, xarm_driver::CallVersion::Response &res);

  sensor_msgs::JointState joint_state_;
  void pubMotorStatus();
  void pubJointState();
  void gripperCallback(const std_msgs::Bool grip);
  void leftGripperCallback(const std_msgs::Bool grip);
  void rightGripperCallback(const std_msgs::Bool grip);

  void singleJointControlCallback(const xarm_driver::SingleJointControl msg);
  void resetCallback(const std_msgs::Empty msg);
  void armCheckTest(const std_msgs::Int16 msg);
  void checkCallback(const std_msgs::Empty msg);


  // 定义了left_arm,right_arm，left_gripper,right_gripper的控制action
  ActionServer arm_as_, grip_as_, left_arm_as_, right_arm_as_, left_grip_as_, right_grip_as_;
  Result arm_result_, grip_result_, left_arm_result_, right_arm_result_, left_grip_result_, right_grip_result_;
  GoalHandle arm_curr_gh_, left_arm_curr_gh_, right_arm_curr_gh_;

  // 单臂的Action回调
  void armGoalCB(GoalHandle gh);
  void armCancelCB(GoalHandle gh);

  // 单臂手爪的Action回调
  void gripGoalCB(GoalHandle gh);
  void gripCancelCB(GoalHandle gh);

  // 左臂的Action回调
  void leftArmGoalCB(GoalHandle gh);
  void leftArmCancelCB(GoalHandle gh);

  // 右臂的Action回调
  void rightArmGoalCB(GoalHandle gh);
  void rightArmCancelCB(GoalHandle gh);

  // 左手爪的Action回调
  void leftGripGoalCB(GoalHandle gh);
  void leftGripCancelCB(GoalHandle gh);

  // 右手爪的Action回调
  void rightGripGoalCB(GoalHandle gh);
  void rightGripCancelCB(GoalHandle gh);

  bool validateJointsNames(GoalHandle& gh, Result& res, const std::vector<std::string> default_names);
  bool validateTrajectory(GoalHandle& gh, Result& res, const std::vector<std::string> default_names);
  void reorderTrajJoints(trajectory_msgs::JointTrajectory& traj, const std::vector<std::string> control_joint_names);
  bool shutdown_requested_, enabled_, connected_;
  bool error_protect_, executing_traj_;

  // 线程相关
  std::atomic<bool> has_goal_, running_, left_arm_interrupt_traj_, right_arm_interrupt_traj_;
  std::thread left_arm_trj_thread_;
  std::thread right_arm_trj_thread_;
  std::thread arm_trj_thread_;


  void trajectoryArmThread();
  void trajectoryLeftArmThread();
  void trajectoryRightArmThread();
  bool followTrajectoryExcute(control_msgs::FollowJointTrajectoryGoal::_trajectory_type &traj, std::string arm_name);
  std::vector<double> interpCubic(double t, double T,
      std::vector<double> p0_pos, std::vector<double> p1_pos,
      std::vector<double> p0_vel, std::vector<double> p1_vel);

  std::vector<double> left_arm_positions_, right_arm_positions_;

  std::mutex tj_mutex_;
  std::condition_variable tj_cv_;

  bool stop_request_;

  // 同步控制左臂舵机运动
  void sycnLeftArmMotors(std::vector<double> rad, std::vector<double> speed);

  // 同步控制右臂舵机运动
  void sycnRightArmMotors(std::vector<double> rad, std::vector<double> speed);


  /****  driver部分 ***************/
  // 串口数据读写
  serial::Serial arm_serial_;
  std::string serial_port_;
  int baud_rate_;
  bool openSerial();

  //  packet handling
  MotorStatus motor_status_;
  XarmData xarm_data_;
  PacketFinder packet_finder_;
  PacketFinder::BufferType data_buffer_;

  ecl::Thread thread_receive_data_;
  ecl::Mutex data_mutex_;
  void dataReceived();
  ecl::Thread thread_check_request_;
  std::mutex data_request_mtx_;
  std::condition_variable data_request_cv_;
  std::thread data_request_th_;
  void dataRequest();

  int error_times_;

  bool checkMotorError();

  void sendCommand(Command command, std::string arm_name);
  ecl::Mutex command_mutex_;  // protection against the user calling the command
                              // functions from multiple threads
  Command command_;  // used to maintain some state about the command history
  Command::Buffer command_buffer_;

  void reset();

  Command xarm_command_;  // used to maintain some state about the command history
  bool readOneMotor(int motor_index);
  serial::Serial   serial_;
  void writeRowData(std::string csv_file, std::vector<double> row_vector);
  std::string log_path_;





};


}  // namespace xbot
#endif  // XARM_ROS_H
