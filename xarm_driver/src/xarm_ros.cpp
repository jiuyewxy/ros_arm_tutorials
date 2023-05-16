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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <xarm_ros.h>
#include <ecl/time.hpp>
#include <thread>

const uint16_t MAX_LOAD = 500;
const uint16_t MAX_SPEED[7] = {1000, 1000, 1000, 1000, 1000, 1000, 1000};
const uint8_t MAX_TEMP = 80;
const uint8_t MIN_VOLTAGE = 85;
const uint8_t MAX_VOLTAGE = 135;
const uint16_t MIN_COUNT_LOC[7] = {1023, 795, 682, 1023, 682, 1023, 2047};
const uint16_t MAX_COUNT_LOC[7] = {3071, 2957, 3412, 3071, 3412, 2388, 2503};

namespace xarm {



void XarmRos::dataReceived() {
  overload_times_ = 0;
  ecl::TimeStamp last_signal_time;
  ecl::Duration timeout(0.1);
  unsigned char buf[256];
  // uint8_t buf;
  uint16_t motor_received_ = 0;
  while (!shutdown_requested_) {
//    ROS_ERROR("XARM ROS : data Received 01 ");

    /*********************
     ** Checking Connection
     **********************/
    if (!arm_serial_.isOpen()) {
      ROS_INFO("Serial closed;");
      openSerial();
    }
    /*********************
     ** Read Incoming
     **********************/
    int n = 0;
    try {
      n = arm_serial_.read(buf, packet_finder_.numberOfDataToRead());
    } catch (const serial::PortNotOpenedException &e) {
      continue;
    }
    if (n == 0) {
      continue;
    }

    bool find_packet = packet_finder_.update(buf, n);


    if (find_packet)
    {
      PacketFinder::BufferType local_buffer;
      packet_finder_.getBuffer(local_buffer);

      packet_finder_.getPayload(data_buffer_);
      data_mutex_.lock();

      while (data_buffer_.size() > 0) {
        if (!motor_status_.deserialise(data_buffer_, xarm_data_,
                                       motor_received_)) {
          break;
        }
      }

      data_mutex_.unlock();
      last_signal_time.stamp();
      // ensure received all motor status info then pub it
      //      ROS_INFO_STREAM("motor_received= "<<int(motor_received_));
      if (motor_received_ == 127) {
        if(!checkMotorError()){
          xarm_state_ = XarmState::ProtectiveStopped;
          // overload_protect_ = true;
          //error_protect_ = true;
         overload_times_ ++;
          ROS_ERROR_STREAM("error ");
        }
       if(overload_times_>8){
         error_protect_ = true;
         overload_times_ = 0;
       }
        pubMotorStatus();
        pubJointState();
        motor_received_ = 0;
      }
    }
//    ROS_ERROR("XARM ROS : data Received 02 ");

  }
}

void XarmRos::dataRequest() {
  ros::Rate r(40);
  while (ros::ok()) {
//    ROS_ERROR("XARM ROS : data request 01 ");
    if (!stop_request_) {
      //        ROS_INFO("requested data...");
      for (uint8_t i = 0; i < 7; i++) {
        sendCommand(Command::setSingleMotorStatusRead(i));
        //      min sleep duration 3000us
        usleep(7000);
      }
    } else {
      continue;
    }
    r.sleep();
  }
//  ROS_ERROR("XARM ROS : data request 02 ");
}
void XarmRos::sendCommand(Command command) {
//  ROS_ERROR("XARM ROS:  send command 01");

  if (!enabled_) {
    ROS_ERROR(
        "The motor controlling is disabled, please check hardware status.");
    return;
  }
  command_mutex_.lock();
  command.resetBuffer(command_buffer_);

  if (!command.serialise(command_buffer_, "right_arm")) {
    ROS_ERROR("command serialise failed.");
  }
  unsigned char checksum = 0;
  for (unsigned int i = 2; i < command_buffer_.size(); i++)
    checksum += command_buffer_[i];

  checksum = ~checksum;
  //  std::cout<<int(checksum)<<std::endl;
  command_buffer_.push_back(checksum);
  arm_serial_.write(&command_buffer_[0], command_buffer_.size());

  // serial_.write((const char *)&command_buffer_[0], command_buffer_.size());

  command_mutex_.unlock();
  //ROS_ERROR("XARM ROS:  send command 02");
}

int test_number = 0;
void XarmRos::sycnControl(std::vector<double> rad, std::vector<double> speed) {
  std::vector<uint16_t> location;
  std::vector<uint16_t> speed_count;
  for (int i = 0; i < 6; i++) {
    location.push_back(rad[i] / 0.00153398 + 2047);
    speed_count.push_back((fabs(speed[i])) / 0.00153398);
  }
  stop_request_ = true;
  sendCommand(Command::setSycnMotorControl(location, speed_count));
  // usleep(20000);
  stop_request_ = false;
  test_number++;
  //  ROS_ERROR_STREAM(test_number<<":  "<<ros::Time::now());
}

// 初始化服务端对象as_,action的名字必须参照ros_controllers.yaml中的参数来写才能与moveit客户端配对
XarmRos::XarmRos(ros::NodeHandle &nh, tf2_ros::Buffer &tf)
    : shutdown_requested_(false),
      enabled_(false),
      connected_(false),
      stop_request_(false),
      nh_(nh),
      tf_(tf),
      joint_set_(DEFAULT_JOINTS.begin(), DEFAULT_JOINTS.end()),
      has_goal_(false),
      running_(false),
      interrupt_traj_(false),
      overload_protect_(false),
      error_protect_(false),
      as_(nh_, "xarm_controller/follow_joint_trajectory",
          boost::bind(&XarmRos::goalCB, this, _1),
          boost::bind(&XarmRos::cancelCB, this, _1), false),
      grip_as_(nh_, "gripper_controller/follow_joint_trajectory",
               boost::bind(&XarmRos::gripGoalCB, this, _1),
               boost::bind(&XarmRos::gripCancelCB, this, _1), false) {
  advertiseTopics();
  subscribeTopics();

  xarm_data_.motor_load.resize(7);
  xarm_data_.motor_temp.resize(7);
  xarm_data_.motor_speed.resize(7);
  xarm_data_.motor_current.resize(7);
  xarm_data_.motor_voltage.resize(7);
  xarm_data_.motor_location.resize(7);
  xarm_data_.motor_work_state.resize(7);


  if (!nh_.getParam("/xarm/xarm_port", serial_port_)) {
    ROS_ERROR(
        "Xarm : no arm device port given on the parameter server (e.g. "
        "/dev/xarm).");
    return;
  }
  // 获取串口波特率
  nh_.param<int>("/xarm/port_baud", port_baud_, 1000000);
  if (openSerial()) {
    connected_ = true;
    ROS_INFO("serial port connected.");
  } else {
    ROS_ERROR("Open arm serial port failed!");
  }

  motor_status_.setArmMode(1);
  joint_state_.name.resize(8);
  joint_state_.name[0] = "arm_1_joint";
  joint_state_.name[1] = "arm_2_joint";
  joint_state_.name[2] = "arm_3_joint";
  joint_state_.name[3] = "arm_4_joint";
  joint_state_.name[4] = "arm_5_joint";
  joint_state_.name[5] = "arm_6_joint";
  joint_state_.name[6] = "gripper_1_joint";
  joint_state_.name[7] = "gripper_2_joint";

  joint_state_.position.resize(8);
  joint_state_.position[0] = 0;
  joint_state_.position[1] = 0;
  joint_state_.position[2] = 0;
  joint_state_.position[3] = 0;
  joint_state_.position[4] = 0;
  joint_state_.position[5] = 0;
  joint_state_.position[6] = 0;
  joint_state_.position[7] = 0;
  joint_state_.velocity.resize(8);
  joint_state_.velocity[0] = 0;
  joint_state_.velocity[1] = 0;
  joint_state_.velocity[2] = 0;
  joint_state_.velocity[3] = 0;
  joint_state_.velocity[4] = 0;
  joint_state_.velocity[5] = 0;
  joint_state_.velocity[6] = 0;
  joint_state_.velocity[7] = 0;
  enable();

  ecl::PushAndPop<unsigned char> stx(2, 0);
  ecl::PushAndPop<unsigned char> etx(1);
  stx.push_back(0xFF);
  stx.push_back(0xFF);
  packet_finder_.configure(stx, etx, 1, 256, 1, true);
  thread_receive_data_.start(&XarmRos::dataReceived, *this);
  thread_check_request_.start(&XarmRos::dataRequest, *this);
  reset();
}
XarmRos::~XarmRos() {
  reset();
  thread_check_request_.join();
  thread_receive_data_.join();
  tj_thread_.join();
  ROS_INFO("XarmRos exited!");
}

bool XarmRos::openSerial() {
  try {
    arm_serial_.setPort(serial_port_);
    arm_serial_.setBaudrate(port_baud_);
    arm_serial_.setStopbits(
        serial::stopbits_one);  // 使用的停止位的数量，默认是一位停止位
    arm_serial_.setFlowcontrol(serial::flowcontrol_none);  //使用的控制流的类型
    arm_serial_.setParity(serial::parity_none);  //奇偶检验的方法
    arm_serial_.setBytesize(serial::eightbits);
    serial::Timeout to = serial::Timeout::simpleTimeout(
        4000);  //单位ms ?需要确定是否和block是一个意思
    arm_serial_.setTimeout(to);  // 设置使用Timeout结构的读写超时
    arm_serial_.open();
    if (arm_serial_.isOpen()) {
      ROS_INFO_ONCE("chassis serial is open");
      return true;
    }
  } catch (serial::IOException &e) {
    arm_serial_.close();
    ROS_ERROR_ONCE("chassis serial connect error");
    return false;
  } catch (...) {
    arm_serial_.close();
    ROS_ERROR_ONCE("chassis serial connect error");
    return false;
  }
}

void XarmRos::startUpdate() { thread_update_.start(&XarmRos::update, *this); }
void XarmRos::update() {
  ros::Rate spin_rate(5);
  while (!shutdown_requested_ && ros::ok()) {
    if (!connected_) {
      ROS_ERROR(
          "Xarm : arm serial port is not connetced, please connect the "
          "Xarm.");
    }
    //    if (!check())
    //      disable();
    //    else
    //      enable();
    spin_rate.sleep();
  }
}

bool XarmRos::check() {
  return checkMotorError() && checkMotorLoad() && checkMotorTemp() &&
         checkMotorVoltage() && checkMotorLocation();
}

bool XarmRos::checkMotorError() {
  for (int i = 1; i < 7; i++) {
    if (xarm_data_.motor_work_state[i] != 0) {
      ROS_ERROR_STREAM("The number " << i
                                     << " motor work error,please check "
                                        "xarm hardware, control "
                                        "disabled! The error status is; " << unsigned( xarm_data_.motor_work_state[i]));
      return false;
    }
  }
  return true;
}

bool XarmRos::checkMotorLoad() {
  for (int i = 0; i < 7; i++) {
    if ((xarm_data_.motor_load[i]) > MAX_LOAD) {
      ROS_ERROR_STREAM(
          "The number "
          << i
          << " motor overload,please check xarm hardware, control disabled! load = " << xarm_data_.motor_load[i]);
      return false;
    }
  }
  return true;
}
bool XarmRos::checkMotorTemp() {
  for (int i = 0; i < 7; i++) {
    if (xarm_data_.motor_temp[i] > MAX_TEMP) {
      ROS_ERROR_STREAM("The number " << i
                                     << " motor temperature is too "
                                        "high,please check xarm hardware, "
                                        "control disabled!");
      return false;
    }
  }
  return true;
}
bool XarmRos::checkMotorVoltage() {
  for (int i = 0; i < 7; i++) {
    if (xarm_data_.motor_voltage[i] > MAX_VOLTAGE ||
        xarm_data_.motor_voltage[i] < MIN_VOLTAGE) {
      ROS_ERROR_STREAM("The number " << i
                                     << " motor voltage is over or too "
                                        "low,please check xarm hardware, "
                                        "control disabled!");
      return false;
    }
  }
  return true;
}
bool XarmRos::checkMotorLocation() {
  for (int i = 0; i < 7; i++) {
    if (xarm_data_.motor_location[i] > MAX_COUNT_LOC[i] ||
        xarm_data_.motor_location[i] < MAX_COUNT_LOC[i]) {
      ROS_ERROR_STREAM("The number " << i
                                     << " motor position error,please "
                                        "check xarm hardware, control "
                                        "disabled!");
      return false;
    }
  }
  return true;
}



void XarmRos::pubMotorStatus() {
  if (ros::ok()) {
//    ROS_ERROR("T 1: Pub Motor");

    xarm_driver::MotorStatus msg;
    xarm_driver::JointLocation joint_valus;
    joint_valus.gripper = (xarm_data_.motor_location[0] - 2047) * M_PI / 2048;
    joint_valus.arm_1 = (xarm_data_.motor_location[1] - 2047) * M_PI / 2048;
    joint_valus.arm_2 = (xarm_data_.motor_location[2] - 2047) * M_PI / 2048;
    joint_valus.arm_3 = (xarm_data_.motor_location[3] - 2047) * M_PI / 2048;
    joint_valus.arm_4 = (xarm_data_.motor_location[4] - 2047) * M_PI / 2048;
    joint_valus.arm_5 = (xarm_data_.motor_location[5] - 2047) * M_PI / 2048;
    joint_valus.arm_6 = (xarm_data_.motor_location[6] - 2047) * M_PI / 2048;

    location_pub_.publish(joint_valus);


    for (int i = 0; i < 7; i++) {
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

void XarmRos::pubJointState() {
//  ROS_ERROR("T 3: Pub jonit state");

  for (int i = 0; i < 6; i++) {
    joint_state_.position[i] =
        (xarm_data_.motor_location[i + 1] - 2047) * M_PI / 2048;

    joint_state_.velocity[i] = xarm_data_.motor_speed[i + 1] * M_PI / 2048;
  }
  // 手爪
  joint_state_.position[6] =
      (xarm_data_.motor_location[0] - 2047) * M_PI / 2048;
  joint_state_.position[7] =
      (xarm_data_.motor_location[0] - 2047) * M_PI / 2048;
  joint_state_.velocity[6] = xarm_data_.motor_speed[0] * M_PI / 2048;
  joint_state_.velocity[7] = xarm_data_.motor_speed[0] * M_PI / 2048;

  // TODO: cal two griper finger state from motor 0
  if (ros::ok()) {
    joint_state_.header.stamp = ros::Time::now();
    joint_state_pub_.publish(joint_state_);
  }
//  ROS_ERROR("T 4: Pub jonit state");
}
void XarmRos::subscribeTopics() {
  simpath_sub_ = nh_.subscribe(std::string("simpath"), 100,
                               &XarmRos::simpathCallback, this);
  gripper_sub_ = nh_.subscribe(std::string("arm/commands/grip"), 100,
                               &XarmRos::gripperCallback, this);
  single_joint_control_sub_ =
      nh_.subscribe(std::string("arm/commands/single_joint_control"), 100,
                    &XarmRos::singleJointControlCallback, this);
  reset_sub_ = nh_.subscribe(std::string("arm/commands/reset"), 100,
                             &XarmRos::resetCallback, this);
}
void XarmRos::advertiseTopics() {
  motor_status_pub_ =
      nh_.advertise<xarm_driver::MotorStatus>("arm/motor_status/data_raw", 100);
  joint_state_pub_ =
      nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
  location_pub_ =  nh_.advertise<xarm_driver::JointLocation>("arm/joint_values", 100);
}
void XarmRos::simpathCallback(const moveit_msgs::RobotTrajectory traj) {
  int control_num = traj.joint_trajectory.points.begin()->positions.size();
}
void XarmRos::gripperCallback(const std_msgs::Bool grip) {
  stop_request_ = true;
  usleep(20000);
  if (grip.data) {
    sendCommand(Command::setSingleMotorControl(0, 2047));
  } else {
    sendCommand(Command::setSingleMotorControl(0, 2503));
  }
  usleep(20000);
  stop_request_ = false;
}

void XarmRos::singleJointControlCallback(
    const xarm_driver::SingleJointControl msg) {
  uint16_t count = msg.rad / 0.00153398 + 2047;
  stop_request_ = true;

  sendCommand(Command::setSingleMotorControl(msg.id, count));
  usleep(20000);
  stop_request_ = false;
}
void XarmRos::resetCallback(const std_msgs::Empty msg) { reset(); }
void XarmRos::reset() {
  stop_request_ = true;
  usleep(20000);
  // arm joints reset
  std::vector<double> init_rad = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float speed[6] = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  sycnControl(init_rad, DEFAULT_ARM_SPEED);
  usleep(20000);

  //  gripper reset
  sendCommand(Command::setSingleMotorControl(0, 2047));

  usleep(20000);
}
}  // namespace xarm
