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

#include "command.h"
#include "ros/ros.h"
namespace xarm {

/*****************************************************************************
** Static variables initialization
*****************************************************************************/

const unsigned char Command::header0 = 0xFF;
const unsigned char Command::header1 = 0xFF;
const uint16_t ALL_SPEED[14] = {200, 200, 200, 200, 200, 200, 200, 200, 200,200, 200, 200, 200, 200 };

//控制单个舵机指令发送
Command Command::setSingleMotorControl(const uint8_t &index,
                                       const uint16_t &location) {
  Command outgoing;
  outgoing.data.index = index;
//  if(7<=int(index)<=13){
//    //ROS_ERROR_STREAM("Wrong single left control");
//  }
  outgoing.data.location = location;
  outgoing.data.command = Command::Write;
  return outgoing;
}

//读取单个舵机的所有状态信息指令发送
Command Command::setSingleMotorStatusRead(const uint8_t &index) {
  //ROS_ERROR("Command test : setSingleMotorStatusRead 01 ");

  Command outgoing;
  outgoing.data.index = index;
  outgoing.data.read_start = 0x38;
  outgoing.data.read_length = 15;
  outgoing.data.command = Command::Read;
  //ROS_ERROR("Command test : setSingleMotorStatusRead 02 ");

  return outgoing;
}

//编号为1~6的手臂关节舵机控制
Command Command::setSycnMotorControl(const std::vector<uint16_t> sycn_location,
                                     const std::vector<uint16_t> sycn_speed) {
  //ROS_ERROR("Command test : setSycnMotorControl 01 ");

  Command outgoing;
  outgoing.data.index = 0xFE;
  for (int i = 0; i < 6; i++) {
    outgoing.data.sycn_location[i] = sycn_location[i];
    outgoing.data.sycn_speed[i] = sycn_speed[i];
  }
  outgoing.data.command = Command::SycnWrite;
  //ROS_ERROR("Command test : setSycnMotorControl 02 ");

  return outgoing;
}

/*****************************************************************************
** Implementation [Serialisation]
*****************************************************************************/
/**
 * 重置缓存区
 */
void Command::resetBuffer(Buffer &buffer) {
  //ROS_ERROR("Command test : reset buffer 01 ");
  buffer.clear();
  buffer.resize(256);
  buffer.push_back(Command::header0);
  buffer.push_back(Command::header1);
  //ROS_ERROR("Command test : reset buffer 02 ");

}

bool Command::serialise(ecl::PushAndPop<unsigned char> &byteStream, std::string arm_name) {

  //ROS_ERROR("Command test : serialise 01 ");

  // need to be sure we don't pass through an emum to the Trans'd buildBytes
  // functions.
  unsigned char cmd = static_cast<unsigned char>(data.command);
  uint8_t size_byte;
  uint8_t index;
  //  uint16_t speed;
  uint16_t time;
  uint8_t address;
  uint8_t label1;
  uint8_t label2;
  uint8_t sycn_cmd(0x83);
  switch (data.command) {

    case Read:
      buildBytes(data.index, byteStream);
      size_byte = 0x04;
      buildBytes(size_byte, byteStream);
      buildBytes(cmd, byteStream);
      buildBytes(data.read_start, byteStream);
      buildBytes(data.read_length, byteStream);
      break;
    case Write:
      buildBytes(data.index, byteStream);
      size_byte = 0x09;
      buildBytes(size_byte, byteStream);
      buildBytes(cmd, byteStream);
      address = 0x2A;
      buildBytes(address, byteStream);
      buildBytes(data.location, byteStream);
      time = 0;
      buildBytes(time, byteStream);
      buildBytes(ALL_SPEED[data.index], byteStream);
      break;
    case RegWrite:
      buildBytes(cmd, byteStream);
      break;
    case Action:
      index = 0xFE;
      buildBytes(index, byteStream);
      size_byte = 0x02;
      buildBytes(size_byte, byteStream);
      buildBytes(cmd, byteStream);
      break;
    case SycnWrite:
      buildBytes(data.index, byteStream);
      size_byte = 0x2E;
      buildBytes(size_byte, byteStream);
      buildBytes(cmd, byteStream);
      label1 = 0x2A;
      buildBytes(label1, byteStream);
      label2 = 0x06;
      buildBytes(label2, byteStream);
      if(arm_name == "right_arm"){
        for (uint8_t i = 1; i < 7; i++) {
          buildBytes(i, byteStream);
          buildBytes(data.sycn_location[i - 1], byteStream);
          time = 0;
          buildBytes(time, byteStream);
          buildBytes(data.sycn_speed[i - 1], byteStream);
          //buildBytes(ALL_SPEED[i],byteStream);
        }
      }else if (arm_name == "left_arm") {
        for (uint8_t i = 7; i < 13; i++) {
          //ROS_ERROR_STREAM("WRONG LEFT ARM 02");
          buildBytes(i, byteStream);
          buildBytes(data.sycn_location[i - 1 - 6], byteStream);
          time = 0;
          buildBytes(time, byteStream);
          buildBytes(data.sycn_speed[i - 1 - 6], byteStream);          
          //buildBytes(ALL_SPEED[i],byteStream);
        }

      }
      break;

      break;
    default:
      return false;
      break;
  }
  //ROS_ERROR("Command test : serialise 02 ");

  return true;

}

}  // namespace xbot
