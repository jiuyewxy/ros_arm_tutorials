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
#include <xarm_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ecl/time.hpp>
#include <thread>
namespace xarm {
void XarmRos::start()
{
  if (running_)
    return;
  ROS_INFO("-------- Starting ActionServer ------");
  running_ = true;
  tj_thread_ = std::thread(&XarmRos::trajectoryThread, this);
  as_.start();
  grip_as_.start();
}


void XarmRos::gripGoalCB(GoalHandle gh){
  ROS_INFO("Received new gripper goal");
  xarm_state_ = XarmState::Running;
  result_.error_code = -100;
  // 判断机械臂是否处于正常状态running
  if(!validateState(gh, result_)){
    ROS_ERROR_STREAM("ERROR:"<<result_.error_string.c_str());
    gh.setRejected(result_, result_.error_string);
  }
  // 判断轨迹路径名称是否有效
  auto const& joints = gh.getGoal()->trajectory.joint_names;
  std::set<std::string> goal_joints(joints.begin(), joints.end());
  std::set<std::string> joint_set(DEFAULT_GRIPPER_JOINTS.begin(),DEFAULT_GRIPPER_JOINTS.end());
  if (goal_joints == joint_set){
    ROS_INFO_STREAM("Joint name valid");
  }else {
    result_.error_code = Result::INVALID_JOINTS;
    result_.error_string = "Invalid joint names for goal\n";
    gh.setRejected(result_, result_.error_string);
  }
  int num = gh.getGoal()->trajectory.points.size();
  double target_joint_value = gh.getGoal()->trajectory.points[num-1].positions[0];
  ROS_INFO_STREAM("target_joint_value: "<<target_joint_value);
  uint16_t count = target_joint_value / 0.00153398 + 2047;
  stop_request_ = true;
  gh.setAccepted();

  sendCommand(Command::setSingleMotorControl(0, count));
  sleep(3);
  gh.setSucceeded();
  usleep(20000);
  stop_request_ = false;

}
void XarmRos::gripCancelCB(GoalHandle gh){

}






// action的回调函数
void XarmRos::goalCB(GoalHandle gh)
{
  ROS_INFO("Received new goal");
  xarm_state_ = XarmState::Running;
  result_.error_code = -100;
  // 判断机械臂是否处于正常状态running
  if(!validateState(gh, result_)){
    ROS_ERROR_STREAM("ERROR:"<<result_.error_string.c_str());
    gh.setRejected(result_, result_.error_string);
  }


  // 判断请求的轨迹的joint名字是否与真实机械臂对应
  if(!validateJointsNames(gh, result_)){
    ROS_ERROR_STREAM("ERROR:"<<result_.error_string.c_str());
    gh.setRejected(result_, result_.error_string);
  }

  // 判断轨迹是否有效
  if(!validateTrajectory(gh, result_)){
    ROS_ERROR_STREAM("ERROR:"<<result_.error_string.c_str());
    gh.setRejected(result_, result_.error_string);
  }

  // 尝试控制机械臂执行
  if(!tryExecute(gh, result_)){
    ROS_ERROR_STREAM("ERROR:"<<result_.error_string.c_str());
    gh.setRejected(result_, result_.error_string);
  }

}

void XarmRos::cancelCB(GoalHandle gh)
{
  interrupt_traj_ = true;
  ROS_INFO("Goal cancelled by client");

  // wait for goal to be interrupted
  std::lock_guard<std::mutex> lock(tj_mutex_);
  Result res;
  res.error_code = -100;
  res.error_string = "Goal cancelled by client";
  gh.setCanceled(res);

  tj_mutex_.unlock();

}


// 判断机器人当前状态
bool XarmRos::validateState(GoalHandle& gh, Result& res)
{
  switch (xarm_state_)
  {
    case XarmState::EmergencyStopped:
      res.error_string = "Robot is emergency stopped";
      return false;

    case XarmState::ProtectiveStopped:
      res.error_string = "Robot is protective stopped";
      return false;

    case XarmState::Error:
      res.error_string = "Robot is not ready, check robot_mode";
      return false;

    case XarmState::Running:
      return true;

    default:
      res.error_string = "Undefined state";
      return false;
  }
}

// 判断机械臂关节的名字是否有效
bool XarmRos::validateJointsNames(GoalHandle& gh, Result& res)
{
  auto goal = gh.getGoal();
  auto const& joints = goal->trajectory.joint_names;
  std::set<std::string> goal_joints(joints.begin(), joints.end());
//  ROS_INFO_STREAM("Goal joints are : ");
//  for (std::set<std::string>::iterator it = goal_joints.begin(); it != goal_joints.end(); it++){
//    ROS_INFO_STREAM("  "<< *it);
//  }
  if (goal_joints == joint_set_){
    ROS_INFO_STREAM("Joint name valid");
    return true;
  }

  res.error_code = Result::INVALID_JOINTS;
  res.error_string = "Invalid joint names for goal\n";
  res.error_string += "Expected: ";
  std::for_each(goal_joints.begin(), goal_joints.end(),
                [&res](std::string joint) { res.error_string += joint + ", "; });
  res.error_string += "\nFound: ";
  std::for_each(joint_set_.begin(), joint_set_.end(), [&res](std::string joint) { res.error_string += joint + ", "; });
  return false;
}

// 判断请求的轨迹是否有效
bool XarmRos::validateTrajectory(GoalHandle& gh, Result& res)
{
  auto goal = gh.getGoal();
  res.error_code = Result::INVALID_GOAL;

  // 至少包含一个轨迹点
  if (goal->trajectory.points.size() < 1)
    return false;

  for (auto const& point : goal->trajectory.points)
  {
    if (point.velocities.size() != joint_set_.size())
    {
      res.error_code = Result::INVALID_GOAL;
      res.error_string = "Received a goal with an invalid number of velocities";
      return false;
    }

    if (point.positions.size() != joint_set_.size())
    {
      res.error_code = Result::INVALID_GOAL;
      res.error_string = "Received a goal with an invalid number of positions";
      return false;
    }

    for (auto const& velocity : point.velocities)
    {
      if (!std::isfinite(velocity))
      {
        res.error_string = "Received a goal with infinities or NaNs in velocity";
        return false;
      }
      if (std::fabs(velocity) > max_velocity_)
      {
        res.error_string =
            "Received a goal with velocities that are higher than max_velocity_ " + std::to_string(max_velocity_);
        return false;
      }
    }
    for (auto const& position : point.positions)
    {
      if (!std::isfinite(position))
      {
        res.error_string = "Received a goal with infinities or NaNs in positions";
        return false;
      }
    }
  }
  return true;
}

// 按照实际关节先后位置重新排列轨迹数据
void XarmRos::reorderTrajJoints(trajectory_msgs::JointTrajectory& traj) {
  std::vector<std::string> actual_joint_names = DEFAULT_JOINTS;
  std::vector<unsigned int> mapping;
  mapping.resize(actual_joint_names.size(), actual_joint_names.size());
  for (unsigned int i = 0; i < traj.joint_names.size(); i++) {
    for (unsigned int j = 0; j < actual_joint_names.size(); j++) {
      if (traj.joint_names[i] == actual_joint_names[j])
        mapping[j] = i;
    }
  }
  traj.joint_names = actual_joint_names;
  std::vector<trajectory_msgs::JointTrajectoryPoint> new_traj;
  for (unsigned int i = 0; i < traj.points.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint new_point;
    for (unsigned int j = 0; j < traj.points[i].positions.size(); j++) {
      new_point.positions.push_back(
          traj.points[i].positions[mapping[j]]);
      new_point.velocities.push_back(
          traj.points[i].velocities[mapping[j]]);
      if (traj.points[i].accelerations.size() != 0)
        new_point.accelerations.push_back(
            traj.points[i].accelerations[mapping[j]]);
    }
    new_point.time_from_start = traj.points[i].time_from_start;
    new_traj.push_back(new_point);
  }
  traj.points = new_traj;
}

bool XarmRos::startPositionsMatch(const trajectory_msgs::JointTrajectory &traj, double eps, Result& res)
{
  // 获取机械臂当前的joint值
  // TOFIX：下面这种方式获取的是否是当前的实时机械臂位置？无真机传电机数据时，先把初始都设为2048
//  for (int i = 0; i < 6; i++) {
//    //joint_state_.position[i] = (xarm_data_.motor_location[i + 1] - 2047) * M_PI / 2048;
//    joint_state_.position[i] = (2048 - 2047) * M_PI / 2048;
//  }

//  for (unsigned int i = 0; i < traj.points[0].positions.size(); i++)
//  {
//    if( fabs(traj.points[0].positions[i] - joint_state_.position[i]) > eps )
//    {
//      ROS_ERROR_STREAM("222"<<traj.points[0].positions[i]<<joint_state_.position[i]);
//      res.error_string = "Start position not match!";
//      return false;
//    }
//  }
  return true;
}

bool XarmRos::tryExecute(GoalHandle& gh, Result& res)
{
  if (!running_)
  {
    res.error_string = "Internal error";
    return false;
  }
  // try_lock:1.如果互斥锁当前未被任何线程锁定，则调用线程将其锁定（从此点开始，直到调用其成员解锁，该线程拥有互斥锁）。
  // 2.如果互斥锁当前被另一个线程锁定，则该函数将失败并返回false，而不会阻塞（调用线程继续执行）。
  // 3.如果互斥锁当前被调用此函数的同一线程锁定，则会产生死锁（具有未定义的行为）。
  if (!tj_mutex_.try_lock())
  {
    ROS_INFO("Received another trajectory");
    interrupt_traj_ = true;
    res.error_string = "Received another trajectory";
    curr_gh_.setAborted(res, res.error_string);
    tj_mutex_.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  // locked here
  curr_gh_ = gh;
  interrupt_traj_ = false;
  has_goal_ = true;
  tj_mutex_.unlock();
  tj_cv_.notify_one();
  return true;
}

// 轨迹控制执行函数
void XarmRos::trajectoryThread(){
  ROS_INFO("Trajectory thread started");
  while (running_){
    std::unique_lock<std::mutex> lk(tj_mutex_);
    if (!tj_cv_.wait_for(lk, std::chrono::milliseconds(100), [&] { return running_ && has_goal_; }))
      continue;
    // 输出重新排序前的轨迹points
//    std::cout<<"--------positions:before reorder----------"<<std::endl;
    auto goal = curr_gh_.getGoal();
    control_msgs::FollowJointTrajectoryGoal::_trajectory_type trajectory;
    trajectory_msgs::JointTrajectory::_points_type::iterator iter;
    trajectory = goal->trajectory;

    // 请求的轨迹关节的顺序是按找首字母先后排列的，可能与机械臂实际关节顺序不一致，所以需要对轨迹重新排列
    reorderTrajJoints(trajectory);
//    ROS_INFO_STREAM(" Traj joint name after reorder: ");
//    for (unsigned int i = 0; i < trajectory.joint_names.size();i++){
//      ROS_INFO_STREAM("--"<<trajectory.joint_names[i]);
//    }
//    // 输出调整顺序后的轨迹
//    std::cout<<"--------positions:after reorder----------"<<std::endl;
//    for(iter= trajectory.points.begin(); iter!=trajectory.points.end(); iter++){
//      for(int i=0;i<iter->positions.size();i++)
//        std::cout<<iter->positions[i]<<" ";
//      std::cout<<std::endl;
//    }
//    std::cout<<"------velocities: after reorder------------"<<std::endl;
//    for(iter=trajectory.points.begin(); iter!=trajectory.points.end(); iter++){
//      for(int i=0;i<iter->velocities.size();i++)
//        std::cout<<iter->velocities[i]<<" ";
//      std::cout<<std::endl;
//    }
//    std::cout<<"--------accelerations: after reorder----------"<<std::endl;
//    for(iter=trajectory.points.begin(); iter!=trajectory.points.end(); iter++){
//      for(int i=0;i<iter->accelerations.size();i++)
//        std::cout<<iter->accelerations[i]<<" ";
//      std::cout<<std::endl;
//    }

    // 在执行轨迹前，还需判断轨迹的起点是否与真实机械臂的起点对应
    // TOFIX:详见startPositionsMatch代码
    if(!startPositionsMatch(trajectory,0.1,result_)){
//      ROS_ERROR_STREAM("ERROR:"<<result_.error_string.c_str());
      curr_gh_.setRejected(result_, result_.error_string);
    }
    ROS_INFO("Trajectory received and accepted");
    curr_gh_.setAccepted();

    // 开始调用控制机械臂跟随轨迹运动的函数followTrajectoryExcute
    if(followTrajectoryExcute(trajectory,interrupt_traj_)){
        ROS_INFO("Trajectory executed successfully");
        result_.error_code = Result::SUCCESSFUL;
        curr_gh_.setSucceeded(result_);       
    }else if(interrupt_traj_){
      ROS_INFO("Trajectory interrupted");
      result_.error_code = -100;
      result_.error_string = "ERROR: Trajectory interrupted";
      curr_gh_.setAborted(result_, result_.error_string);
    }
    else {
      ROS_INFO("Trajectory failed");
      result_.error_code = -100;
      result_.error_string = "ERROR: Follow Trajectory failed";
      curr_gh_.setAborted(result_, result_.error_string);
    }
    has_goal_ = false;
    //overload_protect_ = false;
    overload_times_ = 0;
    error_protect_ = false;
    lk.unlock();
  }
}

// 控制真实机械臂跟随轨迹运动
// TODO：补充完整代码
bool XarmRos::followTrajectoryExcute(control_msgs::FollowJointTrajectoryGoal::_trajectory_type &traj, std::atomic<bool> &is_interupted){
 ROS_INFO("Start controlling ......");
   int control_num = traj.points.size();
   std::vector<double> timestamps;
   std::vector<std::vector<double> > positions, velocities;

   if (traj.points[0].time_from_start.toSec() != 0.) {
     ROS_WARN(
         "Trajectory's first point should be the current position, with time_from_start set to 0.0 - Inserting point in malformed trajectory");
     timestamps.push_back(0.0);
     positions.push_back(joint_state_.position );
     velocities.push_back(joint_state_.velocity );
   }
   for (unsigned int i = 0; i < traj.points.size(); i++) {
     timestamps.push_back(
         traj.points[i].time_from_start.toSec()-0.05);
     positions.push_back(traj.points[i].positions);
     velocities.push_back(traj.points[i].velocities);
   }
   if(!doTraj(timestamps,positions,velocities)){
     return false;

   }
  // TO FIX:在执行轨迹时，需要在规定的时间内完成执行并返回action.setSucceeded，否则move_group客户端会报错
  // 并interrep打断执行,这个时间约是traj时间的加1s。
  // Controller is taking too long to execute trajectory (the expected upper bound for the trajectory execution was 3.653461 seconds). Stopping trajectory.
//  ROS_INFO_STREAM("points size: "<<traj.points.size());
//   ros::Time control_begin = ros::Time::now();
//   int control_num = traj.points.size();
//    ROS_ERROR("control num: %d", control_num);
// //  ros::Duration d1 = traj.points.back().time_from_start/control_num;
//     for (int i = 1; i < control_num; i++) {
//       if(!interrupt_traj_){
//         if(overload_protect_){
//            // 如果过载，则控制机械臂返回上一个状态并返回false
//           ROS_ERROR("Servo overload");
// //          sycnControl(traj.points[i-2].positions, DEFAULT_ARM_SPEED);
//           return false;
//         }
//         //sycnControl(traj.points[i].positions, DEFAULT_ARM_SPEED);
//         sycnControl(traj.points[i].positions, traj.points[i].velocities);
//         double control_time = traj.points[i].time_from_start.toSec() - traj.points[i-1].time_from_start.toSec();
//         double t_d = 0.3;
//         ROS_INFO_STREAM("Control time: "<<control_time);
//         ros::Duration(control_time).sleep();
//       }else {
//         ROS_ERROR_STREAM("interrupt_traj_!!!"<<" :"<<i);
//         return false;

//       }

//     }

    //ROS_ERROR("TEST 111");

    //bool reach_target = false;
    ros::Rate r(5) ;
    int unmatch_times =0;
    while(ros::ok()){

      bool not_match = false;
      for (unsigned int i = 0; i < traj.points[control_num -1].positions.size(); i++)
      {
        if( fabs(traj.points[control_num -1].positions[i] - joint_state_.position[i]) > 0.08 )
        {
//          ROS_ERROR_STREAM("222"<<traj.points[control_num -1].positions[i]<<joint_state_.position[i]);
          not_match = true;
        }
      }
      if(not_match){
        sycnControl(traj.points[control_num -1].positions, DEFAULT_ARM_SPEED);
        unmatch_times++;
        if(unmatch_times>70){
          break;
        }
      }else {
        break;
      }
      r.sleep();
    }

    return true;
}


bool XarmRos::doTraj(std::vector<double> inp_timestamps,
    std::vector<std::vector<double> > inp_positions,
    std::vector<std::vector<double> > inp_velocities) {


  std::chrono::high_resolution_clock::time_point t0, t;
  std::vector<double> positions;
  unsigned int j;

  executing_traj_ = true;
  t0 = std::chrono::high_resolution_clock::now();
  t = t0;
  j = 0;

  while ((inp_timestamps[inp_timestamps.size() - 1]
      >= std::chrono::duration_cast<std::chrono::duration<double>>(t - t0).count())
      and executing_traj_) {
    if(interrupt_traj_ || error_protect_){
      executing_traj_ = false;
      return false;
    }
    while (inp_timestamps[j]
        <= std::chrono::duration_cast<std::chrono::duration<double>>(
            t - t0).count() && j < inp_timestamps.size() - 1) {
      j += 1;
    }
    positions = XarmRos::interpCubic(
        std::chrono::duration_cast<std::chrono::duration<double>>(
            t - t0).count() - inp_timestamps[j - 1],
        inp_timestamps[j] - inp_timestamps[j - 1], inp_positions[j - 1],
        inp_positions[j], inp_velocities[j - 1], inp_velocities[j]);

    sycnControl(positions,inp_velocities[j-1]);

    // oversample with 4 * sample_time
        std::this_thread::sleep_for(std::chrono::milliseconds((int) (50)));
    t = std::chrono::high_resolution_clock::now();
  }

  executing_traj_ = false;
  return true;
}

// 立方差值
std::vector<double> XarmRos::interpCubic(double t, double T,
    std::vector<double> p0_pos, std::vector<double> p1_pos,
    std::vector<double> p0_vel, std::vector<double> p1_vel) {
  /*Returns positions of the joints at time 't' */
  std::vector<double> positions;
  for (unsigned int i = 0; i < p0_pos.size(); i++) {
    double a = p0_pos[i];
    double b = p0_vel[i];
    double c = (-3 * p0_pos[i] + 3 * p1_pos[i] - 2 * T * p0_vel[i]
        - T * p1_vel[i]) / pow(T, 2);
    double d = (2 * p0_pos[i] - 2 * p1_pos[i] + T * p0_vel[i]
        + T * p1_vel[i]) / pow(T, 3);
    positions.push_back(a + b * t + c * pow(t, 2) + d * pow(t, 3));
  }
  return positions;
}

geometry_msgs::PoseStamped XarmRos::goalToArmbaseFrame(
    const geometry_msgs::PoseStamped &goal_pose_msg) {
  std::string global_frame = "world";
  geometry_msgs::PoseStamped goal_pose, global_pose;
  goal_pose = goal_pose_msg;

  // just get the latest available transform... for accuracy they should send
  // goals in the frame of the planner
  goal_pose.header.stamp = ros::Time();

  try {
    tf_.transform(goal_pose_msg, global_pose, global_frame);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
             goal_pose.header.frame_id.c_str(), global_frame.c_str(),
             ex.what());
    return goal_pose_msg;
  }

  return global_pose;
}

bool XarmRos::isQuaternionValid(const geometry_msgs::Quaternion &q) {
  // first we need to check if the quaternion has nan's or infs
  if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w)) {
    ROS_ERROR("Quaternion has nans or infs... discarding as a xarm goal");
    return false;
  }

  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

  // next, we need to check if the length of the quaternion is close to zero
  if (tf_q.length2() < 1e-6) {
    ROS_ERROR("Quaternion has length close to zero... discarding as xarm goal");
    return false;
  }

  // next, we'll normalize the quaternion and check that it transforms the
  // vertical vector correctly
  tf_q.normalize();

  tf2::Vector3 up(0, 0, 1);

  double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

  if (fabs(dot - 1) > 1e-3) {
    ROS_ERROR(
        "Quaternion is invalid... for xarm end the z-axis of the quaternion "
        "must be close to vertical.");
    return false;
  }

  return true;
}

}  // namespace xbot
