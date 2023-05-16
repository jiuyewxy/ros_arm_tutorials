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
#include "xarm_ros_wrapper.h"
namespace xarm{

XarmRosWrapper::XarmRosWrapper(ros::NodeHandle &nh,  std::string serial_port,
int baud_rate)
    : shutdown_requested_(false),
      enabled_(true),
      connected_(false),
      stop_request_(false),
      nh_(nh),
      has_goal_(false),
      running_(false),
      left_arm_interrupt_traj_(false),
      right_arm_interrupt_traj_(false),
      error_protect_(false),
      left_arm_as_(nh_, "left_arm_controller/follow_joint_trajectory", boost::bind(&XarmRosWrapper::leftArmGoalCB, this, _1),
                  boost::bind(&XarmRosWrapper::leftArmCancelCB, this, _1), false),
      right_arm_as_(nh_, "right_arm_controller/follow_joint_trajectory", boost::bind(&XarmRosWrapper::rightArmGoalCB, this, _1),
                  boost::bind(&XarmRosWrapper::rightArmCancelCB, this, _1), false),
      left_grip_as_(nh_, "left_gripper_controller/follow_joint_trajectory", boost::bind(&XarmRosWrapper::leftGripGoalCB, this, _1),
                  boost::bind(&XarmRosWrapper::leftGripCancelCB, this, _1), false),
      right_grip_as_(nh_, "right_gripper_controller/follow_joint_trajectory", boost::bind(&XarmRosWrapper::rightGripGoalCB, this, _1),
                  boost::bind(&XarmRosWrapper::rightGripCancelCB, this, _1), false),
      arm_as_(nh_, "xarm_controller/follow_joint_trajectory",
          boost::bind(&XarmRosWrapper::armGoalCB, this, _1),
          boost::bind(&XarmRosWrapper::armCancelCB, this, _1), false),
      grip_as_(nh_, "gripper_controller/follow_joint_trajectory",
               boost::bind(&XarmRosWrapper::gripGoalCB, this, _1),
               boost::bind(&XarmRosWrapper::gripCancelCB, this, _1), false)
       {
  log_path_ = "/home/xbot";
  nh_.getParam("/xarm/log_path", log_path_);
  ROS_INFO_STREAM("log_path_: "<<log_path_);

  if (!nh_.getParam("/xarm/xarm_port", serial_port_)) {
    ROS_ERROR(
        "Xarm : no arm device port given on the parameter server (e.g. "
        "/dev/xarm).");
    return;
  }
  // ROS_ERROR_STREAM("serial_port_ "<<serial_port_);

  if (!nh_.getParam("/xarm/port_baud", baud_rate_)) {
    ROS_ERROR(
        "Xarm : no baud_rate_ given on the parameter server (e.g. "
        "/dev/port_baud).");
    return;
  }
  // ROS_ERROR_STREAM("baud_rate_ "<<baud_rate_);

  if (!nh_.getParam("/xarm/xarm_mode", xarm_mode_)) {
    ROS_ERROR(
        "Xarm : no arm xarm_mode_ given on the parameter server (e.g. "
        "/dev/xarm_mode).");
    return;
  }
  // ROS_ERROR_STREAM("MODE "<<xarm_mode_);
  // 打开串口
  if (openSerial()) {
    connected_ = true;
    ROS_INFO("serial port connected.");
  } else {
    ROS_ERROR("Open arm serial port failed!");
  }

  // 根据单臂还是双臂进行一些参数设置
  if(!motor_status_.setArmMode(xarm_mode_)){
    ROS_ERROR_STREAM("Wrong xarm_mode param: "<<xarm_mode_<<" . Please use 1 for single arm or 2 for double arm.");
    exit(1);
  }

  if(xarm_mode_ == 1){
    // 开启单臂相关控制action
    arm_as_.start();
    grip_as_.start();
    // 设置存储解码出的舵机数据的大小
    xarm_data_.motor_load.resize(7);
    xarm_data_.motor_temp.resize(7);
    xarm_data_.motor_speed.resize(7);
    xarm_data_.motor_current.resize(7);
    xarm_data_.motor_voltage.resize(7);
    xarm_data_.motor_location.resize(7);
    xarm_data_.motor_work_state.resize(7);
    // 设置joint_state话题的格式
    joint_state_.name = CONTROL_ARM_JOINTS ;
    joint_state_.name.insert(joint_state_.name.end(),CONTROL_GRIP_JOINTS.begin(),CONTROL_GRIP_JOINTS.end());
    joint_state_.position.resize(8);
    joint_state_.velocity.resize(8);


  }else if(xarm_mode_ == 2){
    // 开启双臂控制相关action
    left_arm_as_.start();
    right_arm_as_.start();
    left_grip_as_.start();
    right_grip_as_.start();
    xarm_data_.motor_load.resize(14);
    xarm_data_.motor_temp.resize(14);
    xarm_data_.motor_speed.resize(14);
    xarm_data_.motor_current.resize(14);
    xarm_data_.motor_voltage.resize(14);
    xarm_data_.motor_location.resize(14);
    xarm_data_.motor_work_state.resize(14);
    joint_state_.name.resize(16);
    joint_state_.name = CONTROL_RIGHT_ARM_JOINTS;
    joint_state_.name.insert(joint_state_.name.end(),CONTROL_LEFT_ARM_JOINTS.begin(),CONTROL_LEFT_ARM_JOINTS.end());
    joint_state_.name.insert(joint_state_.name.end(),CONTROL_RIGHT_GRIP_JOINTS.begin(),CONTROL_RIGHT_GRIP_JOINTS.end());
    joint_state_.name.insert(joint_state_.name.end(),CONTROL_LEFT_GRIP_JOINTS.begin(),CONTROL_LEFT_GRIP_JOINTS.end());
    joint_state_.position.resize(16);
    joint_state_.velocity.resize(16);
  }

  // 发布和接收的topics
  advertiseTopics();
  subscribeTopics();

  call_version_ =  nh_.advertiseService(std::string("/arm/version"),&XarmRosWrapper::callVersionSrv,this);


  ecl::PushAndPop<unsigned char> stx(2, 0);
  ecl::PushAndPop<unsigned char> etx(1);
  stx.push_back(0xFF);
  stx.push_back(0xFF);
  packet_finder_.configure(stx, etx, 1, 256, 1, true);
  thread_receive_data_.start(&XarmRosWrapper::dataReceived, *this);
  // thread_check_request_.start(&XarmRosWrapper::dataRequest, *this);
  data_request_th_ = std::thread(&XarmRosWrapper::dataRequest,this);
  data_request_mtx_.unlock();
  data_request_cv_.notify_all();
  // 恢复到初始位置
  reset();
  sleep(2);
  ROS_INFO("\033[1;32m[√] Xbot-Arm  has initialized successfully! You can start now ...... \033[0m");


}

bool XarmRosWrapper::callVersionSrv(xarm_driver::CallVersion::Request &req, xarm_driver::CallVersion::Response &res){
  res.version = "1.0.0";
  return true;
}

 void XarmRosWrapper::halt(){
   //thread_check_request_.join();
   //thread_receive_data_.join();
 }

XarmRosWrapper::~XarmRosWrapper() {
  reset();
  // thread_check_request_.join();
  thread_receive_data_.join();
  data_request_th_.join();
  ROS_INFO("XarmRos exited!");
}

void XarmRosWrapper::subscribeTopics() {
  gripper_sub_ = nh_.subscribe(std::string("arm/commands/grip"), 100,
                               &XarmRosWrapper::gripperCallback, this);
  left_gripper_sub_ = nh_.subscribe(std::string("left_arm/commands/grip"), 100,
                               &XarmRosWrapper::leftGripperCallback, this);
  right_gripper_sub_ = nh_.subscribe(std::string("right_arm/commands/grip"), 100,
                               &XarmRosWrapper::rightGripperCallback, this);
  single_joint_control_sub_ =
      nh_.subscribe(std::string("arm/commands/single_joint_control"), 100,
                    &XarmRosWrapper::singleJointControlCallback, this);
  reset_sub_ = nh_.subscribe(std::string("arm/commands/reset"), 100,
                             &XarmRosWrapper::resetCallback, this);

  check_test_ = nh_.subscribe(std::string("arm/right_check"), 100,
                              &XarmRosWrapper::checkCallback, this);
  if(xarm_mode_ == 1){
    arm_check_ = nh_.subscribe(std::string("/arm/check_test"), 100,
                               &XarmRosWrapper::armCheckTest, this);
  }
}

void XarmRosWrapper::advertiseTopics() {
  motor_status_pub_ =
      nh_.advertise<xarm_driver::MotorStatus>("arm/motor_status/data_raw", 100);
  joint_state_pub_ =
      nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
  location_pub_ =  nh_.advertise<xarm_driver::JointLocation>("arm/joint_values", 100);
  left_location_pub_ =  nh_.advertise<xarm_driver::JointLocation>("left_arm/joint_values", 100);
  right_location_pub_ =  nh_.advertise<xarm_driver::JointLocation>("right_arm/joint_values", 100);
}

void XarmRosWrapper::gripperCallback(const std_msgs::Bool grip) {
  data_request_mtx_.try_lock();
  stop_request_ = true;
  usleep(20000);
  if (grip.data) {
    sendCommand(Command::setSingleMotorControl(0, 2047), "right_arm");
  } else {
    sendCommand(Command::setSingleMotorControl(0, 2503), "right_arm");
  }
  usleep(20000);
  data_request_mtx_.unlock();
  stop_request_ = false;
  data_request_cv_.notify_all();
}

void XarmRosWrapper::leftGripperCallback(const std_msgs::Bool grip) {
  data_request_mtx_.try_lock();
  stop_request_ = true;
  usleep(20000);
  if (grip.data) {
    sendCommand(Command::setSingleMotorControl(13, 2047), "left_arm");
  } else {
    sendCommand(Command::setSingleMotorControl(13, 2503), "left_arm");
  }
  usleep(20000);
  data_request_mtx_.unlock();
  stop_request_ = false;
  data_request_cv_.notify_all();
}
void XarmRosWrapper::rightGripperCallback(const std_msgs::Bool grip) {
  data_request_mtx_.try_lock();
  stop_request_ = true;
  usleep(20000);
  if (grip.data) {
    sendCommand(Command::setSingleMotorControl(0, 2047), "right_arm");
  } else {
    sendCommand(Command::setSingleMotorControl(0, 2503), "right_arm");
  }
  usleep(20000);
  data_request_mtx_.unlock();
  stop_request_ = false;
  data_request_cv_.notify_all();
}

void XarmRosWrapper::singleJointControlCallback(
    const xarm_driver::SingleJointControl msg) {
  ROS_INFO("Single arm control ......");
  uint16_t count = msg.rad / 0.00153398 + 2047;
  data_request_mtx_.try_lock();
  stop_request_ = true;
  usleep(20000);
  if(msg.id <= 6){
    sendCommand(Command::setSingleMotorControl(msg.id, count), "right_arm");
  }else if(msg.id >6){
    sendCommand(Command::setSingleMotorControl(msg.id, count), "left_arm");
  }
  usleep(20000);
  data_request_mtx_.unlock();
  stop_request_ = false;
  data_request_cv_.notify_all();
}

// 单臂和桌面机械臂测试检测接口，输入为循环测试的次数
void XarmRosWrapper::armCheckTest(const std_msgs::Int16 msg){
  ROS_INFO("Start arm check and test!");
  std::string csv_file = log_path_+"/arm_check_test.csv";
  std::vector<double> count_deviation = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  std::vector<double> check_rad = {-0.9,-0.9, 1.93, -0.95, 1.3, 1.2};
  //std::ofstream outFile(csv_file, std::ios::app);

  for(int i=0;i<msg.data+1;i++){
    ROS_INFO_STREAM("TEST: "<<i);
    sycnRightArmMotors(check_rad, DEFAULT_ARM_SPEED );
    sleep(12);
    ROS_INFO("--- Results of check arm motors: ---");
    if(fabs(joint_state_.position[0] - check_rad[0])< 0.05){
      ROS_INFO("\033[1;32m[√] arm_1_joint check\033[0m");
    }else {
      ROS_INFO("\033[1;31m[×] arm_1_joint check\033[0m");
      ROS_INFO_STREAM("   —— current location is "<<joint_state_.position[0]<<", but should be -1.6!");

    }
    if(fabs(joint_state_.position[1] - check_rad[1])< 0.05){
      ROS_INFO("\033[1;32m[√] arm_2_joint check\033[0m");
    }else {
      ROS_INFO("\033[1;31m[×] arm_2_joint check\033[0m");
      ROS_INFO_STREAM("   —— current location is "<<joint_state_.position[1]<<", but should be -1.0 !");

    }
    if(fabs(joint_state_.position[2] - check_rad[2])< 0.05){
      ROS_INFO("\033[1;32m[√] arm_3_joint check\033[0m");
    }else {
      ROS_INFO("\033[1;31m[×] arm_3_joint check\033[0m");
      ROS_INFO_STREAM("   —— current location is "<<joint_state_.position[2]<<", but should be 1.7 !");

    }
    if(fabs(joint_state_.position[3] - check_rad[3])< 0.05){
      ROS_INFO("\033[1;32m[√] arm_4_joint check\033[0m");
    }else {
      ROS_INFO("\033[1;31m[×] arm_4_joint check\033[0m");
      ROS_INFO_STREAM("   —— current location is "<<joint_state_.position[3]<<", but should be -1.7 !");

    }
    if(fabs(joint_state_.position[4] - check_rad[4])< 0.05){
      ROS_INFO("\033[1;32m[√] arm_5_joint check\033[0m");
    }else {
      ROS_INFO("\033[1;31m[×] arm_5_joint check\033[0m");
      ROS_INFO_STREAM("   —— current location is "<<joint_state_.position[4]<<", but should be 2.0 !");

    }
    if(fabs(joint_state_.position[5] - check_rad[5])< 0.05){
      ROS_INFO("\033[1;32m[√] arm_6_joint check\033[0m");
    }else {
      ROS_INFO("\033[1;31m[×] arm_6_joint check\033[0m");
      ROS_INFO_STREAM("   —— current location is "<<joint_state_.position[5]<<", but should be -1.2!");

    }
    uint16_t count = 0.65 / 0.00153398 + 2047;
    data_request_mtx_.try_lock();
    stop_request_ = true;
    usleep(20000);
    sendCommand(Command::setSingleMotorControl(0, count), "right_arm");
    usleep(20000);
    data_request_mtx_.unlock();
    stop_request_ = false;
    data_request_cv_.notify_all();
    sleep(4);
    if(fabs(joint_state_.position[6] - (0.65))< 0.05){
      ROS_INFO("\033[1;32m[√] gripper_open check\033[0m");
    }else {
      ROS_INFO("\033[1;31m[×] gripper_open check\033[0m");
    }

    // write count_deviation to csv_file

    count_deviation[0] = xarm_data_.motor_location[0] - count;
    for(int i=1;i<7;i++){
      if(check_rad[i-1]>=0){
        count_deviation[i] = xarm_data_.motor_location[i] - uint16_t(check_rad[i-1]/ 0.00153398 + 2047);
      }else{
        count_deviation[i] = -xarm_data_.motor_location[i] + uint16_t(check_rad[i-1]/ 0.00153398 + 2047);
      }

    }
    writeRowData(csv_file,count_deviation);
    data_request_mtx_.try_lock();
    stop_request_ = true;
    usleep(20000);
    sendCommand(Command::setSingleMotorControl(0, 2047), "right_arm");
    usleep(20000);
    data_request_mtx_.unlock();
    stop_request_ = false;
    data_request_cv_.notify_all();
    sleep(4);
    if(fabs(joint_state_.position[6] - 0.0)< 0.05){
      ROS_INFO("\033[1;32m[√] gripper_close check\033[0m");
    }else {
      ROS_INFO("\033[1;31m[×] gripper_close check\033[0m");
    }
    sleep(2);
    reset();
    sleep(10);
//    for(int i=0;i<7;i++){
//      count_deviation[i] = fabs(xarm_data_.motor_location[i] - uint16_t(0.0/ 0.00153398 + 2047));
//    }
//    writeRowData(csv_file,count_deviation);

  }
  ROS_INFO("--- check arm done! ---");
}
void XarmRosWrapper::writeRowData(std::string csv_file, std::vector<double> row_vector)
{
  int size = row_vector.size();
  std::ofstream outFile(csv_file, std::ios::app);
  if (outFile.is_open())
  {
    for (int i = 0; i < size; i++)
    {
      outFile << row_vector[i] << ',';
    }
    outFile << std::endl;
  }
  else
  {
    std::cout << "Fail to open the CSV file! " << std::endl;
  }
  outFile.close();
}
void XarmRosWrapper::resetCallback(const std_msgs::Empty msg) { reset(); }

void XarmRosWrapper::checkCallback(const std_msgs::Empty msg) {
  // check test right arm
  ROS_INFO("Test right arm control");
  std::vector<double> LINIT_PICK_JOINTS = {1.0, 0, 1.52, 1.82, 0, -1.4};
  std::vector<double> RIGHT_PICK_JOINTS = {-1.7, 0.8, -1.52, 1.8, -0.3, -1.4};
  std::vector<double> HOME_JOINTS = {0, 0, 0, 0, 0, 0};
  std::vector<double> joint_position = HOME_JOINTS;
  for (int m =0; m<50; m++){
    ROS_INFO_STREAM(" M : " << m);
  for (int i = 0; i < 15; ++i) {
    for (int j =0 ; j<6;j++){
      joint_position[j] += RIGHT_PICK_JOINTS[j]/15.0;
    }
    sycnRightArmMotors(joint_position,DEFAULT_ARM_SPEED);
    ros::Duration(0.4).sleep();
  }
  sleep(3);

  for (int i = 0; i < 15; ++i) {
    for (int j =0 ; j<6;j++){
      joint_position[j] -= RIGHT_PICK_JOINTS[j]/15.0;
    }
    sycnRightArmMotors(joint_position,DEFAULT_ARM_SPEED);
    ros::Duration(0.4).sleep();
  }
  sleep(3);
  }

  ROS_INFO("Done right arm control");



}

// 单臂控制的action回调函数
void XarmRosWrapper::armGoalCB(GoalHandle gh){
  ROS_INFO("--- Received new goal for left arm --- ");
  arm_result_.error_code = -100;

  // 判断请求的轨迹的joint名字是否与真实机械臂对应
  if(!validateJointsNames(gh, arm_result_, CONTROL_ARM_JOINTS)){
    ROS_ERROR_STREAM("ERROR:"<<arm_result_.error_string.c_str());
    gh.setRejected(arm_result_, arm_result_.error_string);
  }

  // 判断轨迹是否有效
  if(!validateTrajectory(gh, arm_result_, CONTROL_ARM_JOINTS)){
    ROS_ERROR_STREAM("ERROR:"<<arm_result_.error_string.c_str());
    gh.setRejected(arm_result_, arm_result_.error_string);
  }

  arm_curr_gh_ = gh;

  // 开启新的线程控制左臂开始执行
  arm_trj_thread_ = std::thread(&XarmRosWrapper::trajectoryArmThread, this);
  arm_trj_thread_.detach();
}
void XarmRosWrapper::armCancelCB(GoalHandle gh){
  // 因为右臂的舵机编号和单臂的一致，所以控制桌面机械臂和单臂时，采用右臂的控制代码
  right_arm_interrupt_traj_ = true;
  ROS_INFO("Goal cancelled by client");
  left_arm_result_.error_code = -100;
  left_arm_result_.error_string = "Goal cancelled by client";
  gh.setCanceled(left_arm_result_);
}


void XarmRosWrapper::gripGoalCB(GoalHandle gh){
  ROS_INFO("Received new gripper goal");
  grip_result_.error_code = -100;

//  // 判断请求的轨迹的joint名字是否与真实机械臂对应
//  if(!validateJointsNames(gh, grip_result_, CONTROL_GRIP_JOINTS)){
//    ROS_ERROR_STREAM("ERROR:"<<grip_result_.error_string.c_str());
//    gh.setRejected(grip_result_, grip_result_.error_string);
//  }

//  // 判断轨迹是否有效
//  if(!validateTrajectory(gh, grip_result_, CONTROL_GRIP_JOINTS)){
//    ROS_ERROR_STREAM("ERROR:"<<grip_result_.error_string.c_str());
//    gh.setRejected(grip_result_, grip_result_.error_string);
//  }


  int num = gh.getGoal()->trajectory.points.size();
  double target_joint_value = gh.getGoal()->trajectory.points[num-1].positions[0];
  ROS_INFO_STREAM("gripper target_joint_value: "<<target_joint_value);
  uint16_t count = target_joint_value / 0.00153398 + 2047;
  gh.setAccepted();
  data_request_mtx_.try_lock();
  stop_request_ = true;
  usleep(20000);
  sendCommand(Command::setSingleMotorControl(0, count), "right_arm");
  usleep(20000);
  data_request_mtx_.unlock();
  stop_request_ = false;
  data_request_cv_.notify_all();
  sleep(3);
  if(fabs(joint_state_.position[6] - target_joint_value ) > 0.05){
    data_request_mtx_.try_lock();
    stop_request_ = true;
    usleep(20000);
    sendCommand(Command::setSingleMotorControl(0, count), "right_arm");
    usleep(20000);
    data_request_mtx_.unlock();
    stop_request_ = false;
    data_request_cv_.notify_all();
  }
  gh.setSucceeded();
}
void XarmRosWrapper::gripCancelCB(GoalHandle gh){

}



// 左臂控制的action回调函数
void XarmRosWrapper::leftArmGoalCB(GoalHandle gh){
  ROS_INFO("--- Received new goal for left arm --- ");
  left_arm_result_.error_code = -100;

  // 判断请求的轨迹的joint名字是否与真实机械臂对应
  if(!validateJointsNames(gh, left_arm_result_, CONTROL_LEFT_ARM_JOINTS)){
    ROS_ERROR_STREAM("ERROR:"<<left_arm_result_.error_string.c_str());
    gh.setRejected(left_arm_result_, left_arm_result_.error_string);
  }

  // 判断轨迹是否有效
  if(!validateTrajectory(gh, left_arm_result_, CONTROL_LEFT_ARM_JOINTS)){
    ROS_ERROR_STREAM("ERROR:"<<left_arm_result_.error_string.c_str());
    gh.setRejected(left_arm_result_, left_arm_result_.error_string);
  }

  left_arm_curr_gh_ = gh;

  // 开启新的线程控制左臂开始执行
  left_arm_trj_thread_ = std::thread(&XarmRosWrapper::trajectoryLeftArmThread, this);
  left_arm_trj_thread_.detach();

}
void XarmRosWrapper::leftArmCancelCB(GoalHandle gh){
  left_arm_interrupt_traj_ = true;
  ROS_INFO("Goal cancelled by client");
  left_arm_result_.error_code = -100;
  left_arm_result_.error_string = "Goal cancelled by client";
  gh.setCanceled(left_arm_result_);
}


// 右臂控制的action回调函数
void XarmRosWrapper::rightArmGoalCB(GoalHandle gh){
  ROS_INFO("--- Received new goal for right arm --- ");
  right_arm_result_.error_code = -100;

  // 判断请求的轨迹的joint名字是否与真实机械臂对应
  if(!validateJointsNames(gh, right_arm_result_, CONTROL_RIGHT_ARM_JOINTS)){
    ROS_ERROR_STREAM("ERROR:"<<right_arm_result_.error_string.c_str());
    gh.setRejected(right_arm_result_, right_arm_result_.error_string);
  }

  // 判断轨迹是否有效
  if(!validateTrajectory(gh, right_arm_result_, CONTROL_RIGHT_ARM_JOINTS)){
    ROS_ERROR_STREAM("ERROR:"<<right_arm_result_.error_string.c_str());
    gh.setRejected(right_arm_result_, right_arm_result_.error_string);
  }
  right_arm_curr_gh_ = gh;
  // 开启新的线程控制右臂开始执行
  right_arm_trj_thread_ = std::thread(&XarmRosWrapper::trajectoryRightArmThread, this);
  right_arm_trj_thread_.detach();

}
void XarmRosWrapper::rightArmCancelCB(GoalHandle gh){
  right_arm_interrupt_traj_ = true;
  ROS_INFO("Goal cancelled by client");
  right_arm_result_.error_code = -100;
  right_arm_result_.error_string = "Goal cancelled by client";
  gh.setCanceled(right_arm_result_);
}


// 左手爪控制的action回调函数
void XarmRosWrapper::leftGripGoalCB(GoalHandle gh){
  ROS_INFO("Received new gripper goal");
  left_grip_result_.error_code = -100;

  // 判断请求的轨迹的joint名字是否与真实机械臂对应
  if(!validateJointsNames(gh, left_grip_result_, CONTROL_LEFT_GRIP_JOINTS)){
    ROS_ERROR_STREAM("ERROR:"<<left_grip_result_.error_string.c_str());
    gh.setRejected(left_grip_result_, left_grip_result_.error_string);
  }

  // 判断轨迹是否有效
  if(!validateTrajectory(gh, left_grip_result_, CONTROL_LEFT_GRIP_JOINTS)){
    ROS_ERROR_STREAM("ERROR:"<<grip_result_.error_string.c_str());
    gh.setRejected(left_grip_result_, left_grip_result_.error_string);
  }


  int num = gh.getGoal()->trajectory.points.size();
  double target_joint_value = gh.getGoal()->trajectory.points[num-1].positions[0];
  ROS_INFO_STREAM("target_joint_value: "<<target_joint_value);
  uint16_t count = target_joint_value / 0.00153398 + 2047;
  gh.setAccepted();
  data_request_mtx_.try_lock();
  stop_request_ = true;
  usleep(20000);
  sendCommand(Command::setSingleMotorControl(0, 2047), "right_arm");
  usleep(20000);
  data_request_mtx_.unlock();
  stop_request_ = false;
  data_request_cv_.notify_all();
  sleep(3);
  if(fabs(joint_state_.position[14] - target_joint_value ) > 0.05){
    data_request_mtx_.try_lock();
    stop_request_ = true;
    usleep(20000);
    sendCommand(Command::setSingleMotorControl(13, count), "left_arm");
    usleep(20000);
    data_request_mtx_.unlock();
    stop_request_ = false;
    data_request_cv_.notify_all();
  }
  gh.setSucceeded();

}
void XarmRosWrapper::leftGripCancelCB(GoalHandle gh){
}


// 右手爪控制的action回调函数
void XarmRosWrapper::rightGripGoalCB(GoalHandle gh){
  ROS_INFO("Received new gripper goal");
  right_grip_result_.error_code = -100;

  // 判断请求的轨迹的joint名字是否与真实机械臂对应
  if(!validateJointsNames(gh, right_grip_result_, CONTROL_RIGHT_GRIP_JOINTS)){
    ROS_ERROR_STREAM("ERROR:"<<right_grip_result_.error_string.c_str());
    gh.setRejected(right_grip_result_, right_grip_result_.error_string);
  }

  // 判断轨迹是否有效
  if(!validateTrajectory(gh, right_grip_result_, CONTROL_RIGHT_GRIP_JOINTS)){
    ROS_ERROR_STREAM("ERROR:"<<right_grip_result_.error_string.c_str());
    gh.setRejected(right_grip_result_, right_grip_result_.error_string);
  }
  int num = gh.getGoal()->trajectory.points.size();
  double target_joint_value = gh.getGoal()->trajectory.points[num-1].positions[0];
  ROS_INFO_STREAM("target_joint_value: "<<target_joint_value);
  uint16_t count = target_joint_value / 0.00153398 + 2047;
  data_request_mtx_.try_lock();
  stop_request_ = true;
  gh.setAccepted();
  usleep(20000);
  sendCommand(Command::setSingleMotorControl(0, count), "right_arm");
  usleep(20000);
  data_request_mtx_.unlock();
  stop_request_ = false;
  data_request_cv_.notify_all();
  sleep(3);
  if(fabs(joint_state_.position[12] - target_joint_value ) > 0.05){
    data_request_mtx_.try_lock();
    stop_request_ = true;
    usleep(20000);
    sendCommand(Command::setSingleMotorControl(0, count), "right_arm");
    data_request_mtx_.unlock();
    stop_request_ = false;
    data_request_cv_.notify_all();
  }
  gh.setSucceeded();
}
void XarmRosWrapper::rightGripCancelCB(GoalHandle gh){
}

// 单臂控制的执行
void XarmRosWrapper::trajectoryArmThread(){
  ROS_INFO("--- Trajectory excute thread started for arm ---");
  auto goal = arm_curr_gh_.getGoal();
  control_msgs::FollowJointTrajectoryGoal::_trajectory_type trajectory;
  trajectory_msgs::JointTrajectory::_points_type::iterator iter;
  trajectory = goal->trajectory;
  // 请求的轨迹关节的顺序是按找首字母先后排列的，可能与机械臂实际关节顺序不一致，所以需要对轨迹重新排列
  reorderTrajJoints(trajectory, CONTROL_ARM_JOINTS);
  ROS_INFO_STREAM(" Traj joint name after reorder: ");
  for (unsigned int i = 0; i < trajectory.joint_names.size();i++){
    ROS_INFO_STREAM("--"<<trajectory.joint_names[i]);
  }


  ROS_INFO("-- Trajectory received and accepted ---");
  arm_curr_gh_.setAccepted();

  // 开始调用控制机械臂跟随轨迹运动的函数followTrajectoryExcute
  // 因为右臂的舵机编号和单臂的一致，所以控制桌面机械臂和单臂时，采用右臂的控制代码
  if(followTrajectoryExcute(trajectory,"right_arm")){
    ROS_INFO("Trajectory executed successfully");
    arm_result_.error_code = Result::SUCCESSFUL;
    arm_curr_gh_.setSucceeded(arm_result_);
  }else if(right_arm_interrupt_traj_){
    ROS_INFO("Trajectory interrupted");
    arm_result_.error_code = -100;
    arm_result_.error_string = "ERROR: Trajectory interrupted";
    arm_curr_gh_.setAborted(arm_result_, arm_result_.error_string);
  }
  else {
    ROS_INFO("Trajectory failed");
    arm_result_.error_code = -100;
    arm_result_.error_string = "ERROR: Follow Trajectory failed";
    arm_curr_gh_.setAborted(arm_result_, arm_result_.error_string);
  }
  right_arm_interrupt_traj_ = false;
  error_protect_ = false;
}


// 左臂轨迹控制执行函数
void XarmRosWrapper::trajectoryLeftArmThread(){
  ROS_INFO("--- Trajectory excute thread started for left arm ---");
  auto goal = left_arm_curr_gh_.getGoal();
  control_msgs::FollowJointTrajectoryGoal::_trajectory_type trajectory;
  trajectory_msgs::JointTrajectory::_points_type::iterator iter;
  trajectory = goal->trajectory;
  // 请求的轨迹关节的顺序是按找首字母先后排列的，可能与机械臂实际关节顺序不一致，所以需要对轨迹重新排列
  reorderTrajJoints(trajectory, CONTROL_LEFT_ARM_JOINTS);
  ROS_INFO_STREAM(" Traj joint name after reorder: ");
  for (unsigned int i = 0; i < trajectory.joint_names.size();i++){
    ROS_INFO_STREAM("--"<<trajectory.joint_names[i]);
  }

  ROS_INFO("-- Trajectory received and accepted ---");
  left_arm_curr_gh_.setAccepted();

  // 开始调用控制机械臂跟随轨迹运动的函数followTrajectoryExcute
  if(followTrajectoryExcute(trajectory,"left_arm")){
    ROS_INFO("Trajectory executed successfully");
    left_arm_result_.error_code = Result::SUCCESSFUL;
    left_arm_curr_gh_.setSucceeded(left_arm_result_);
  }else if(left_arm_interrupt_traj_){
    ROS_INFO("Trajectory interrupted");
    left_arm_result_.error_code = -100;
    left_arm_result_.error_string = "ERROR: Trajectory interrupted";
    left_arm_curr_gh_.setAborted(left_arm_result_, left_arm_result_.error_string);
  }
  else {
    ROS_INFO("Trajectory failed");
    left_arm_result_.error_code = -100;
    left_arm_result_.error_string = "ERROR: Follow Trajectory failed";
    left_arm_curr_gh_.setAborted(left_arm_result_, left_arm_result_.error_string);
  }
  left_arm_interrupt_traj_ = false;
  error_protect_ = false;
}

// 右臂轨迹控制执行函数
void XarmRosWrapper::trajectoryRightArmThread(){
  ROS_INFO("--- Trajectory excute thread started for right arm ---");
  auto goal = right_arm_curr_gh_.getGoal();
  control_msgs::FollowJointTrajectoryGoal::_trajectory_type trajectory;
  trajectory_msgs::JointTrajectory::_points_type::iterator iter;
  trajectory = goal->trajectory;
  // 请求的轨迹关节的顺序是按找首字母先后排列的，可能与机械臂实际关节顺序不一致，所以需要对轨迹重新排列
  reorderTrajJoints(trajectory, CONTROL_RIGHT_ARM_JOINTS);
  ROS_INFO_STREAM(" Traj joint name after reorder: ");
  for (unsigned int i = 0; i < trajectory.joint_names.size();i++){
    ROS_INFO_STREAM("--"<<trajectory.joint_names[i]);
  }
//  // 输出调整顺序后的轨迹
//  std::cout<<"--------positions:after reorder----------"<<std::endl;
//  for(iter= trajectory.points.begin(); iter!=trajectory.points.end(); iter++){
//    for(int i=0;i<iter->positions.size();i++)
//      std::cout<<iter->positions[i]<<" ";
//    std::cout<<std::endl;
//  }
//  std::cout<<"------velocities: after reorder------------"<<std::endl;
//  for(iter=trajectory.points.begin(); iter!=trajectory.points.end(); iter++){
//    for(int i=0;i<iter->velocities.size();i++)
//      std::cout<<iter->velocities[i]<<" ";
//    std::cout<<std::endl;
//  }
//  std::cout<<"--------accelerations: after reorder----------"<<std::endl;
//  for(iter=trajectory.points.begin(); iter!=trajectory.points.end(); iter++){
//    for(int i=0;i<iter->accelerations.size();i++)
//      std::cout<<iter->accelerations[i]<<" ";
//    std::cout<<std::endl;
//  }

  ROS_INFO("-- Trajectory received and accepted ---");
  right_arm_curr_gh_.setAccepted();

  // 开始调用控制机械臂跟随轨迹运动的函数followTrajectoryExcute
  if(followTrajectoryExcute(trajectory,"right_arm")){
    ROS_INFO("Trajectory executed successfully");
    right_arm_result_.error_code = Result::SUCCESSFUL;
    right_arm_curr_gh_.setSucceeded(right_arm_result_);
  }else if(right_arm_interrupt_traj_){
    ROS_INFO("Trajectory interrupted");
    right_arm_result_.error_code = -100;
    right_arm_result_.error_string = "ERROR: Trajectory interrupted";
    right_arm_curr_gh_.setAborted(right_arm_result_, right_arm_result_.error_string);
  }
  else {
    ROS_INFO("Trajectory failed");
    right_arm_result_.error_code = -100;
    right_arm_result_.error_string = "ERROR: Follow Trajectory failed";
    right_arm_curr_gh_.setAborted(right_arm_result_, right_arm_result_.error_string);
  }
  stop_request_ = false;
  data_request_cv_.notify_all();
  right_arm_interrupt_traj_ = false;
  error_protect_ = false;
}



bool  XarmRosWrapper::followTrajectoryExcute(control_msgs::FollowJointTrajectoryGoal::_trajectory_type &traj, std::string arm_name){
  ROS_INFO("--- Start controlling ---");
  int control_num = traj.points.size();

  std::vector<double> timestamps;
  std::vector<std::vector<double> > positions, velocities;
  // 先加入零点位置
  timestamps.push_back(
      traj.points[0].time_from_start.toSec());
  positions.push_back(traj.points[0].positions);
  velocities.push_back(traj.points[0].velocities);


  for (unsigned int i = 1; i < traj.points.size(); i++) {
    timestamps.push_back(
        fabs(traj.points[i].time_from_start.toSec()-0.05));
    positions.push_back(traj.points[i].positions);
    velocities.push_back(traj.points[i].velocities);
  }


  std::chrono::high_resolution_clock::time_point t0, t;
  std::vector<double> positions_cb;
  unsigned int j;

  executing_traj_ = true;
  t0 = std::chrono::high_resolution_clock::now();
  t = t0;
  j = 0;

  while ((timestamps[timestamps.size() - 1]
      >= std::chrono::duration_cast<std::chrono::duration<double>>(t - t0).count())
      and executing_traj_) {
    if(arm_name =="left_arm" && (left_arm_interrupt_traj_ || error_protect_)){
      //left_arm_interrupt_traj_ = false;
      return false;
    }
    if(arm_name =="right_arm" && (right_arm_interrupt_traj_ || error_protect_)){
      //right_arm_interrupt_traj_ = false;
      return false;
    }
    while (timestamps[j]
        <= std::chrono::duration_cast<std::chrono::duration<double>>(
            t - t0).count() && j < timestamps.size() - 1) {
      j += 1;
    }
    positions_cb = interpCubic(
        std::chrono::duration_cast<std::chrono::duration<double>>(
            t - t0).count() - timestamps[j - 1],
        timestamps[j] - timestamps[j - 1], positions[j - 1],
        positions[j], velocities[j - 1], velocities[j]);
    if(arm_name == "left_arm"){
      sycnLeftArmMotors(positions_cb,velocities[j-1]);
    }else if (arm_name == "right_arm"){
      sycnRightArmMotors(positions_cb,velocities[j-1]);
    }

    // oversample with 4 * sample_time
        std::this_thread::sleep_for(std::chrono::milliseconds((int) (50)));
    t = std::chrono::high_resolution_clock::now();
  }

//     for (int i = 1; i < control_num -1; i++) {
//           if(arm_name =="left_arm" && (left_arm_interrupt_traj_ || error_protect_)){
//             left_arm_interrupt_traj_ = false;
//             return false;
//           }
//           if(arm_name =="right_arm" && (right_arm_interrupt_traj_ || error_protect_)){
//             right_arm_interrupt_traj_ = false;
//             return false;
//           }

//           if(arm_name == "left_arm"){
//              sycnLeftArmMotors(traj.points[i].positions, traj.points[i].velocities);
//           }else if (arm_name == "right_arm"){
//              sycnRightArmMotors(traj.points[i].positions, traj.points[i].velocities);
//           }
//           //sycnControl(traj.points[i].positions, DEFAULT_ARM_SPEED);
//           //sycnControl(traj.points[i].positions, traj.points[i].velocities);
//           double control_time = fabs(traj.points[i].time_from_start.toSec() - traj.points[i-1].time_from_start.toSec());
//           //ROS_INFO_STREAM("Control time: "<<control_time);
//           ros::Duration(control_time).sleep();
//       }
//     if(arm_name == "left_arm"){
//       sycnLeftArmMotors(traj.points[control_num -1].positions, DEFAULT_ARM_SPEED);
//     }else if (arm_name == "right_arm"){
//       sycnRightArmMotors(traj.points[control_num -1].positions, DEFAULT_ARM_SPEED);
//     }

  executing_traj_ = false;

  ros::Rate r(5) ;
  int unmatch_times =0;
  while(ros::ok()){
    bool not_match = false;
    for (unsigned int i = 0; i < traj.points[control_num -1].positions.size(); i++)
    {
      if(arm_name == "left_arm"){
        if( fabs(traj.points[control_num -1].positions[i] - joint_state_.position[i+6]) > 0.08 )
        {
          not_match = true;
        }
      }else if (arm_name == "right_arm") {
        if( fabs(traj.points[control_num -1].positions[i] - joint_state_.position[i]) > 0.08 )
        {
          not_match = true;
        }

      }
    }
    if(not_match){
      if(arm_name == "left_arm"){
        sycnLeftArmMotors(traj.points[control_num -1].positions, DEFAULT_ARM_SPEED);
      }else if (arm_name == "right_arm"){
        sycnRightArmMotors(traj.points[control_num -1].positions, DEFAULT_ARM_SPEED);
      }
      //sycnControl(traj.points[control_num -1].positions, DEFAULT_ARM_SPEED);
      unmatch_times++;
      if(unmatch_times > 70){
        return false;
      }
    }else {
      break;
    }
    r.sleep();
  }

  return true;
}


// 立方差值
std::vector<double> XarmRosWrapper::interpCubic(double t, double T,
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


// 按照实际关节先后位置重新排列轨迹数据
void XarmRosWrapper::reorderTrajJoints(trajectory_msgs::JointTrajectory& traj, const std::vector<std::string> control_joint_names) {
  std::vector<std::string> actual_joint_names = control_joint_names;
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

// 判断机械臂关节的名字是否有效
bool XarmRosWrapper::validateJointsNames(GoalHandle& gh, Result& res, const std::vector<std::string> default_names)
{
  auto goal = gh.getGoal();
  auto const& joints = goal->trajectory.joint_names;
  std::set<std::string> goal_joints(joints.begin(), joints.end());
  ROS_INFO_STREAM("Goal joints are : ");
  for (std::set<std::string>::iterator it = goal_joints.begin(); it != goal_joints.end(); it++){
    ROS_INFO_STREAM("  "<< *it);
  }
  std::set<std::string> joint_set(default_names.begin(), default_names.end());
  if (goal_joints == joint_set){
    ROS_INFO_STREAM("Joint name valid");
    return true;
  }

  res.error_code = Result::INVALID_JOINTS;
  res.error_string = "Invalid joint names for goal\n";
  res.error_string += "Expected: ";
  std::for_each(goal_joints.begin(), goal_joints.end(),
                [&res](std::string joint) { res.error_string += joint + ", "; });
  res.error_string += "\nFound: ";
  std::for_each(joint_set.begin(), joint_set.end(), [&res](std::string joint) { res.error_string += joint + ", "; });
  return false;
}

// 判断请求的轨迹是否有效
bool XarmRosWrapper::validateTrajectory(GoalHandle& gh, Result& res, const std::vector<std::string> default_names)
{
  auto goal = gh.getGoal();
  res.error_code = Result::INVALID_GOAL;

  // 至少包含一个轨迹点
  if (goal->trajectory.points.size() < 1)
    return false;

  for (auto const& point : goal->trajectory.points)
  {
    if (point.velocities.size() != default_names.size())
    {
      res.error_code = Result::INVALID_GOAL;
      res.error_string = "Received a goal with an invalid number of velocities";
      return false;
    }

    if (point.positions.size() != default_names.size())
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
      if (std::fabs(velocity) > 0.5)
      {
        res.error_string =
            "Received a goal with velocities that are higher than max_velocity_ " + std::to_string(0.5);
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


void XarmRosWrapper::pubMotorStatus() {
  if (ros::ok()) {
//    ROS_ERROR("T 1: Pub Motor");
    if(xarm_mode_ == 1){
      xarm_driver::JointLocation joint_valus;
      joint_valus.gripper = (xarm_data_.motor_location[0] - 2047) * M_PI / 2048;
      joint_valus.arm_1 = (xarm_data_.motor_location[1] - 2047) * M_PI / 2048;
      joint_valus.arm_2 = (xarm_data_.motor_location[2] - 2047) * M_PI / 2048;
      joint_valus.arm_3 = (xarm_data_.motor_location[3] - 2047) * M_PI / 2048;
      joint_valus.arm_4 = (xarm_data_.motor_location[4] - 2047) * M_PI / 2048;
      joint_valus.arm_5 = (xarm_data_.motor_location[5] - 2047) * M_PI / 2048;
      joint_valus.arm_6 = (xarm_data_.motor_location[6] - 2047) * M_PI / 2048;
      location_pub_.publish(joint_valus);
    }
    if(xarm_mode_ == 2){
      xarm_driver::JointLocation joint_valus1;
      joint_valus1.gripper = (xarm_data_.motor_location[0] - 2047) * M_PI / 2048;
      joint_valus1.arm_1 = (xarm_data_.motor_location[1] - 2047) * M_PI / 2048;
      joint_valus1.arm_2 = (xarm_data_.motor_location[2] - 2047) * M_PI / 2048;
      joint_valus1.arm_3 = (xarm_data_.motor_location[3] - 2047) * M_PI / 2048;
      joint_valus1.arm_4 = (xarm_data_.motor_location[4] - 2047) * M_PI / 2048;
      joint_valus1.arm_5 = (xarm_data_.motor_location[5] - 2047) * M_PI / 2048;
      joint_valus1.arm_6 = (xarm_data_.motor_location[6] - 2047) * M_PI / 2048;
      right_location_pub_.publish(joint_valus1);
      xarm_driver::JointLocation joint_valus2;
      joint_valus2.gripper = (xarm_data_.motor_location[13] - 2047) * M_PI / 2048;
      joint_valus2.arm_1 = (xarm_data_.motor_location[7] - 2047) * M_PI / 2048;
      joint_valus2.arm_2 = (xarm_data_.motor_location[8] - 2047) * M_PI / 2048;
      joint_valus2.arm_3 = (xarm_data_.motor_location[9] - 2047) * M_PI / 2048;
      joint_valus2.arm_4 = (xarm_data_.motor_location[10] - 2047) * M_PI / 2048;
      joint_valus2.arm_5 = (xarm_data_.motor_location[11] - 2047) * M_PI / 2048;
      joint_valus2.arm_6 = (xarm_data_.motor_location[12] - 2047) * M_PI / 2048;
      left_location_pub_.publish(joint_valus2);
    }



    xarm_driver::MotorStatus msg;

    for (int i = 0; i < xarm_data_.motor_location.size(); i++) {
      msg.error.push_back(xarm_data_.motor_work_state[i]);
      msg.location.push_back(xarm_data_.motor_location[i]);
      msg.load.push_back(xarm_data_.motor_load[i] / 10.0);
      msg.current.push_back(xarm_data_.motor_current[i] / 10.0);
      msg.temperature.push_back(xarm_data_.motor_temp[i]);
      msg.voltage.push_back(xarm_data_.motor_voltage[i] / 10.0);
      msg.speed.push_back(xarm_data_.motor_speed[i]);
    }
    motor_status_pub_.publish(msg);
//    ROS_ERROR("T 2: Pub Motor");
  }
}

void XarmRosWrapper::pubJointState() {
//  ROS_ERROR("T 3: Pub jonit state");

  if(xarm_mode_ == 1){
    for (int i = 0; i < 6; i++) {
      joint_state_.position[i] =
        (xarm_data_.motor_location[i + 1] - 2047) * M_PI / 2048;

      joint_state_.velocity[i] = xarm_data_.motor_speed[i + 1]*0.087/180*3.1415926 ;
    }
    // 手爪
    joint_state_.position[6] =
      (xarm_data_.motor_location[0] - 2047) * M_PI / 2048;
    joint_state_.position[7] =
      (xarm_data_.motor_location[0] - 2047) * M_PI / 2048;
    joint_state_.velocity[6] = xarm_data_.motor_speed[0] *0.087/180*3.1415926;
    joint_state_.velocity[7] = xarm_data_.motor_speed[0] *0.087/180*3.1415926;
  }else if (xarm_mode_ == 2){
    for (int i = 0; i < 12; i++) {
      joint_state_.position[i] =
        (xarm_data_.motor_location[i + 1] - 2047) * M_PI / 2048;

      joint_state_.velocity[i] = xarm_data_.motor_speed[i + 1] *0.087/180*3.1415926;
    }
    // 手爪
    joint_state_.position[12] =
      (xarm_data_.motor_location[0] - 2047) * M_PI / 2048;
    joint_state_.position[13] = joint_state_.position[12];
    joint_state_.velocity[12] = xarm_data_.motor_speed[0] *0.087/180*3.1415926;
    joint_state_.velocity[13] = joint_state_.velocity[12];

    joint_state_.position[14] =
      (xarm_data_.motor_location[13] - 2047) * M_PI / 2048;
    joint_state_.position[15] = joint_state_.position[14];
    joint_state_.velocity[14] = xarm_data_.motor_speed[13] *0.087/180*3.1415926;
    joint_state_.velocity[15] = joint_state_.velocity[14];

  }

  if (ros::ok()) {
    joint_state_.header.stamp = ros::Time::now();
    joint_state_pub_.publish(joint_state_);
  }
//  ROS_ERROR("T 4: Pub jonit state");
}

}  // namespace xarm
