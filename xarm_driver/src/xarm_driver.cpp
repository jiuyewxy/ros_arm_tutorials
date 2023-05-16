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
#include <ecl/time.hpp>
#include <thread>
#include "xarm_ros_wrapper.h"
#include <sys/time.h>
namespace xarm {

bool PacketFinder::checkSum() {
  unsigned int packet_size(buffer.size());
  unsigned char cs(0);
  for (unsigned int i = 2; i < packet_size - 1; i++) {
    cs += buffer[i];
    //ROS_ERROR_STREAM("CS " << int(cs));
    cs = ~cs;
  }
  return (cs == buffer[packet_size - 1]) ? false : true;
  return true;
}

// 打开串口
bool XarmRosWrapper::openSerial() {
  try {
    arm_serial_.setPort(serial_port_);
    arm_serial_.setBaudrate(baud_rate_);
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

void XarmRosWrapper::reset() {
  ROS_INFO("RESET ARM");
//  stop_request_ = true;
//  usleep(20000);
  // arm joints reset
  std::vector<double> init_rad = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  sycnRightArmMotors(init_rad, DEFAULT_ARM_SPEED );
  //usleep(20000);

  //  right gripper reset
  data_request_mtx_.try_lock();
  stop_request_ = true;
  usleep(20000);
  sendCommand(Command::setSingleMotorControl(0, 2047), "right_arm");
  usleep(20000);
  data_request_mtx_.unlock();
  stop_request_ = false;
  data_request_cv_.notify_all();

  if(xarm_mode_ == 2){
    sycnLeftArmMotors(init_rad, DEFAULT_ARM_SPEED );
    usleep(20000);
  }
  data_request_mtx_.try_lock();
  stop_request_ = true;
  usleep(20000);
  sendCommand(Command::setSingleMotorControl(13, 2047), "left_arm");
  usleep(20000);
  data_request_mtx_.unlock();
  stop_request_ = false;
  data_request_cv_.notify_all();
  //stop_request_ = false;
}

void XarmRosWrapper::dataReceived() {
  error_times_ = 0;
  ecl::TimeStamp last_signal_time;
  ecl::Duration timeout(0.1);
  unsigned char buf[256];
  // uint8_t buf;
  uint16_t motor_received_ = 0;
  while (!shutdown_requested_) {
//    ROS_ERROR("XARM ROS : data Received 01 ");
    if (!arm_serial_.isOpen()) {
      ROS_INFO("Serial closed;");
      openSerial();
    }

    int n = 0;
    try {
      n = arm_serial_.read(buf, packet_finder_.numberOfDataToRead());
    } catch (const serial::PortNotOpenedException &e) {
      continue;
    }
    //ROS_ERROR_STREAM("data <<"<<int(buf[3]));

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
      //ROS_ERROR_STREAM("data_buffer_.size() <<"<<data_buffer_.size());

      while (data_buffer_.size() > 0) {

        if (!motor_status_.deserialise(data_buffer_, xarm_data_,
                                       motor_received_)) {
          break;
        }
      }

      data_mutex_.unlock();
      last_signal_time.stamp();
      // ensure received all motor status info then pub it
       //ROS_INFO_STREAM("motor_received= "<<int(motor_received_));
      if(xarm_mode_ == 2){
        if (motor_received_ == 16383) {
          if(!checkMotorError()){
            error_times_ ++;
            ROS_ERROR_STREAM("error ");
           }
          if(error_times_>8){
           error_protect_ = true;
           error_times_ = 0;
          }
          pubMotorStatus();
          pubJointState();
          motor_received_ = 0;
        }
      }
      if(xarm_mode_ == 1){
        if (motor_received_ == 127) {
          if(!checkMotorError()){
            error_times_ ++;
            ROS_ERROR_STREAM("error ");
           }
          if(error_times_>8){
           error_protect_ = true;
           error_times_ = 0;
          }
          pubMotorStatus();
          pubJointState();
          motor_received_ = 0;
        }
      }
    }
   //ROS_ERROR("XARM ROS : data Received 02 ");

  }
}

void XarmRosWrapper::dataRequest() {
  ros::Rate r(40);
  while (connected_) {
    if(xarm_mode_ == 1){
      for (uint8_t i = 0; i < 7; i++) {
        std::unique_lock <std::mutex> lck(data_request_mtx_);
        if (!data_request_cv_.wait_for(lck, std::chrono::milliseconds(1), [&] { return !stop_request_; }))
          continue;
        sendCommand(Command::setSingleMotorStatusRead(i), "right_arm");
        usleep(14000);
//        if (!stop_request_) {
//          sendCommand(Command::setSingleMotorStatusRead(i), "right_arm");
//          usleep(14000);
//        }else {
//          continue;
//        }
      }
    }else if (xarm_mode_ == 2){
      for (uint8_t i = 0; i < 14; i++) {
        std::unique_lock <std::mutex> lck(data_request_mtx_);
        if (!data_request_cv_.wait_for(lck, std::chrono::milliseconds(1), [&] { return !stop_request_; }))
          continue;
        sendCommand(Command::setSingleMotorStatusRead(i), "right_arm");
        usleep(7000);

//        if (!stop_request_) {
//          sendCommand(Command::setSingleMotorStatusRead(i), "right_arm");
//          usleep(7000);
//        }else {
//          continue;
//        }
      }
    }
  r.sleep();
  }
}

void XarmRosWrapper::sendCommand(Command command, std::string arm_name) {
  //ROS_ERROR("XARM driver:  send command 01");
  if (!enabled_) {
    ROS_ERROR(
        "The motor controlling is disabled, please check hardware status.");
    return;
  }
  command_mutex_.lock();
  command.resetBuffer(command_buffer_);

  if (!command.serialise(command_buffer_, arm_name)) {
    ROS_ERROR("command serialise failed.");
  }
  unsigned char checksum = 0;
  for (unsigned int i = 2; i < command_buffer_.size(); i++)
    checksum += command_buffer_[i];

  checksum = ~checksum;
  //  std::cout<<int(checksum)<<std::endl;
  command_buffer_.push_back(checksum);
  arm_serial_.write(&command_buffer_[0], command_buffer_.size());
//  if(command_buffer_.size()!=8){
//    std::ofstream outFile(log_path_+"/log.txt", std::ios::app|std::ios::out|std::ios::binary);
//    if (outFile.is_open())
//    {
//      ros::Time::now();
//      const time_t t = time(NULL);
//      struct tm* systemtime = localtime(&t);
//      std::string time_stamp = std::to_string(1900 + systemtime->tm_year) + "-" +
//          std::to_string(1 + systemtime->tm_mon) + "-" + std::to_string(systemtime->tm_mday) + "-" +
//          std::to_string(systemtime->tm_hour) + ":" + std::to_string(systemtime->tm_min)+":"+std::to_string(systemtime->tm_sec);
//      struct timeval tv;
//      gettimeofday(&tv,NULL);
//      outFile<<time_stamp<<":"<<(unsigned long long)tv.tv_usec / 1000 <<"  : ";
//      for(int i=0;i<command_buffer_.size();i++){
//        outFile<<std::hex<<(unsigned int)command_buffer_[i];
//      }
//      outFile << std::endl;
//    }
//    else
//    {
//      std::cout << "Fail to open the CSV file! " << std::endl;
//    }
//    outFile.close();

//  }


  // serial_.write((const char *)&command_buffer_[0], command_buffer_.size());

  command_mutex_.unlock();
  //ROS_ERROR("XARM driver:  send command 02");
}


bool XarmRosWrapper::checkMotorError() {
  if(xarm_mode_ ==1){
    for (int i = 1; i < 7; i++) {
      if (xarm_data_.motor_work_state[i] != 0) {
        ROS_ERROR_STREAM("The number " << i
                                     << " motor work error,please check "
                                        "xarm hardware, control "
                                        "disabled! The error status is; " << unsigned( xarm_data_.motor_work_state[i]));
        return false;
      }
    }
  }
  if(xarm_mode_ == 2){
    for (int i = 1; i < 13; i++) {
      if (xarm_data_.motor_work_state[i] != 0) {
        ROS_ERROR_STREAM("The number " << i
                                     << " motor work error,please check "
                                        "xarm hardware, control "
                                        "disabled! The error status is; " << unsigned( xarm_data_.motor_work_state[i]));
        return false;
      }
    }
  }
  return true;
}


// 同步控制左臂舵机运动
void XarmRosWrapper::sycnLeftArmMotors(std::vector<double> rad, std::vector<double> speed) {
  std::vector<uint16_t> location;
  std::vector<uint16_t> speed_count;
  for (int i = 0; i < 6; i++) {
    location.push_back(rad[i] / 0.00153398 + 2047);
    speed_count.push_back((fabs(speed[i])) / 0.00153398);
  }
//  stop_request_ = true;
//  usleep(20000);
  data_request_mtx_.try_lock();
  stop_request_ = true;
  usleep(20000);
  sendCommand(Command::setSycnMotorControl(location, speed_count), "left_arm");
  usleep(20000);
  data_request_mtx_.unlock();
  stop_request_ = false;
  data_request_cv_.notify_all();
  //  ROS_ERROR_STREAM(test_number<<":  "<<ros::Time::now());
}

// 同步控制右臂臂舵机运动
void XarmRosWrapper::sycnRightArmMotors(std::vector<double> rad, std::vector<double> speed) {
  std::vector<uint16_t> location;
  std::vector<uint16_t> speed_count;
  for (int i = 0; i < 6; i++) {
    location.push_back(rad[i] / 0.00153398 + 2047);
    speed_count.push_back(fabs(speed[i]) / 0.00153398);
  }
  data_request_mtx_.try_lock();
  stop_request_ = true;
  usleep(20000);
  sendCommand(Command::setSycnMotorControl(location, speed_count), "right_arm");
  //ros::Duration(0.5).sleep();
  usleep(20000);
  data_request_mtx_.unlock();
  stop_request_ = false;
  data_request_cv_.notify_all();

}

// 发送读取motor_index号舵机的请求并对返回的数据帧进行解码
bool XarmRosWrapper::readOneMotor(int motor_index){

  unsigned char buf[256];
  uint16_t motor_received_ = 0;
  Command::Buffer command_buffer;
  Command command = xarm_command_.setSingleMotorStatusRead(motor_index);
  command.resetBuffer(command_buffer);

  if (!command.serialise(command_buffer, "right_arm")) {
    ROS_ERROR("command serialise failed.");
    return false;
  }

  unsigned char checksum = 0;
  for (unsigned int i = 2; i < command_buffer.size(); i++)
    checksum += command_buffer[i];

  checksum = ~checksum;
  command_buffer.push_back(checksum);
  try {
    serial_.write(&command_buffer[0],command_buffer.size());

  }  catch (serial::IOException &e)
  {
      serial_.close();
      std::cout<<"ERROR:chassis serial connect error!"<<std::endl;
      return false;
  }
  catch(...)
  {
      serial_.close();
      std::cout<<"ERROR:chassis serial connect error!"<<std::endl;
      return false;
  }
  // data_lenth 表示应答的数据镇长度
  size_t data_length;

  bool success = serial_.waitReadable();
  if(!success){
    ROS_ERROR_STREAM("waitReadable failed ");
    return false;
  }
  data_length = serial_.available();
  if (data_length != 21){
    ROS_ERROR_STREAM("Length =  "<<data_length);

    ROS_ERROR_STREAM("ERROR: Wrong data length returned in motor:"<<motor_index);
    return false;
  }
  /*********************
   ** Read Incoming
   **********************/
 for (int i = 0; i <data_length ;i++){
  int n =0;
  try {
    n = serial_.read(buf, 1);    //读取一个字节
  } catch (const serial::PortNotOpenedException &e) {
      return false;
  }

  bool find_packet = packet_finder_.update(buf, n);
  if (find_packet)  // this clears packet finder's buffer and transfers
                    // important bytes into it
  {
    PacketFinder::BufferType local_buffer;
    packet_finder_.getBuffer(local_buffer);

    packet_finder_.getPayload(data_buffer_);
    while (data_buffer_.size() > 0) {
      if (!motor_status_.deserialise(data_buffer_, xarm_data_,
                                     motor_received_)) {
        break;
      }
    }

  }
}
 return true;

}






/*** ****************************************************/
}
