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

#ifndef XARM_ROS_H
#define XARM_ROS_H
#include <actionlib/server/simple_action_server.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf2_ros/transform_listener.h>
#include <xarm_driver/MotorStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <ecl/devices.hpp>
#include <ecl/threads.hpp>
#include "command.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "xarm_driver/SingleJointControl.h"
#include <serial/serial.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "xarm_driver/JointLocation.h"
#include "xarm_ros_wrapper.h"

namespace xarm{

// 真实机械臂的默认关节命名

static const std::vector<std::string> DEFAULT_JOINTS = { "arm_1_joint", "arm_2_joint", "arm_3_joint",
                                                         "arm_4_joint",      "arm_5_joint",       "arm_6_joint" };

static const std::vector<std::string> DEFAULT_GRIPPER_JOINTS = {"gripper_1_joint", "gripper_2_joint"};

//static const std::vector<double> DEFAULT_ARM_SPEED = {0.2, 0.2, 0.2, 0.2, 0.25, 0.25};


// 枚举定义xarm机械臂的状态
enum class XarmState
{
  Running,           // 正常运行
  Error,             // 出现未知错误
  EmergencyStopped,  // 紧急停止
  ProtectiveStopped  // 保护性停止
};



// xarm的ros封装
class XarmRos {
 public:
  XarmRos(ros::NodeHandle &nh, tf2_ros::Buffer &tf);
  ~XarmRos();
  void start();
  //  复位所有舵机到初始位置
  void reset();
  //  always check state
  void startUpdate();
  bool check();
  bool checkMotorError();
  bool checkMotorTemp();
  // 判断当前的负载是否超过安全值
  bool checkMotorLoad();
  bool checkMotorVoltage();
  bool checkMotorLocation();

  bool isEnabled() const { return enabled_; }
  inline bool enable() { enabled_ = true; } /**< Enable controlling motors. **/
  inline bool disable() {
    enabled_ = false;
  } /**< Disable controlling motors. **/

  //  shotdown
  void shutdown() { shutdown_requested_ = true; }
  bool isShutdown() const { return shutdown_requested_; }

  //  the serial port is connected?
  bool is_connected() const { return connected_; }

 private:
  // 定义action的服务端、目标控制句柄以及action的反馈
  typedef control_msgs::FollowJointTrajectoryAction Action;
  typedef control_msgs::FollowJointTrajectoryResult Result;
  typedef actionlib::ServerGoalHandle<Action> GoalHandle;
  typedef actionlib::ActionServer<Action> ActionServer;

  ros::NodeHandle nh_;
  ActionServer as_,grip_as_;
  Result result_,grip_result_;
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
  void gripGoalCB(GoalHandle gh);
  void gripCancelCB(GoalHandle gh);
  XarmState xarm_state_;

  std::set<std::string> joint_set_;
  // max_velocity_等可修改的参数，可以通过param设置，专门写一个ros param解析的函数。
  double max_velocity_ = 0.35;
  bool validateState(GoalHandle& gh, Result& res);
  bool validateJointsNames(GoalHandle& gh, Result& res);
  bool validateTrajectory(GoalHandle& gh, Result& res);
  bool tryExecute(GoalHandle& gh, Result& res);
  bool doTraj(std::vector<double> inp_timestamps,
      std::vector<std::vector<double> > inp_positions,
      std::vector<std::vector<double> > inp_velocities);
  std::vector<double> interpCubic(double t, double T,
      std::vector<double> p0_pos, std::vector<double> p1_pos,
      std::vector<double> p0_vel, std::vector<double> p1_vel);
  bool startPositionsMatch(const trajectory_msgs::JointTrajectory &traj, double eps, Result& res);
  void reorderTrajJoints(trajectory_msgs::JointTrajectory& traj);
  void trajectoryThread();
  bool followTrajectoryExcute(control_msgs::FollowJointTrajectoryGoal::_trajectory_type &traj, std::atomic<bool> &is_interupted);

  bool shutdown_requested_, enabled_, connected_;
  std::string serial_port_;
  int port_baud_;

  // 线程相关
  GoalHandle curr_gh_;
  std::atomic<bool> has_goal_, running_, interrupt_traj_, overload_protect_, error_protect_;
  std::thread tj_thread_;
  std::mutex tj_mutex_;
  std::condition_variable tj_cv_;

  int overload_times_;

  //  every bit indicate a motor data received state exclude the uppest bit
  uint8_t motor_data_received_;
  // 7 motor error
  uint8_t motor_error_[7];
  //  7 motor rad
  float motor_rad_[7];

  //  packet handling
  MotorStatus motor_status_;
  XarmData xarm_data_;
  //ecl::Serial serial_;
  PacketFinder packet_finder_;
  PacketFinder::BufferType data_buffer_;
  // 新加的串口数据读写
  serial::Serial arm_serial_;
  int serial_baud_;
  bool openSerial();
  bool executing_traj_;



  //  commands
  void sendCommand(Command command);
  ecl::Mutex command_mutex_;  // protection against the user calling the command
                              // functions from multiple threads
  Command command_;  // used to maintain some state about the command history
  Command::Buffer command_buffer_;

  ecl::Thread thread_update_;
  void update();
  // 发指令查询所有舵机的状态信息
  bool stop_request_;
  ecl::Thread thread_check_request_;
  void dataRequest();

  // 同步控制所有舵机的目标位置
  void sycnControl(std::vector<double> rad, std::vector<double> speed);

  //  the thread func for receive data
  ecl::Thread thread_receive_data_;
  ecl::Mutex data_mutex_;
  void dataReceived();

  void advertiseTopics();
  void subscribeTopics();
  //  Last validation about the goal control_count and speed befor send it to
  //  motor exacute, return false if not in secure district
  bool validateControlCount(const uint16_t *control_count);
  //  publish xarm 7 motor status and joint state
  ros::Publisher motor_status_pub_, joint_state_pub_, location_pub_;
  //  subscribe the trajectory validated from simulation
  ros::Subscriber simpath_sub_;
  ros::Subscriber gripper_sub_, single_joint_control_sub_, reset_sub_;

  tf2_ros::Buffer &tf_;
//  void executeCb(const xarm_driver::ArmToPoseGoalConstPtr &goal);
  bool isQuaternionValid(const geometry_msgs::Quaternion &q);
  geometry_msgs::PoseStamped goalToArmbaseFrame(
      const geometry_msgs::PoseStamped &goal_pose_msg);
  void simpathCallback(const moveit_msgs::RobotTrajectory traj);
  void gripperCallback(const std_msgs::Bool grip);
  void singleJointControlCallback(const xarm_driver::SingleJointControl msg);
  void resetCallback(const std_msgs::Empty msg);
  void pubMotorStatus();

  sensor_msgs::JointState joint_state_;
  void pubJointState();
};
}  // namespace xbot
#endif  // XARM_ROS_H
