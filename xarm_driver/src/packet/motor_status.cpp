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
#include "packet/motor_status.h"
#include "ros/ros.h"
namespace xarm {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool MotorStatus::serialise(ecl::PushAndPop<unsigned char>& byteStream, std::string arm_name) {
  return true;
}
bool MotorStatus::deserialise(ecl::PushAndPop<unsigned char>& byteStream,
                              XarmData& data_, uint16_t& received_motor) {

  //  舵机ID值
  uint8_t motor_index;

  buildVariable(motor_index, byteStream);
  //ROS_ERROR_STREAM("Motor index"<<int(motor_index));

  if(arm_mode_ == 1){
    if(motor_index>6) return false;
  //  std::cout << "[BEFORE]the motor received id is: " << (int)received_motor
  //            << std::endl;
    if (motor_index < 7) {
      received_motor = received_motor | uint8_t(std::pow(2, motor_index));
    }
  }
  if(arm_mode_ == 2){
    if(motor_index>13) return false;
  //  std::cout << "[BEFORE]the motor received id is: " << (int)received_motor
  //            << std::endl;
    if (motor_index < 14) {
      received_motor = received_motor | uint16_t(std::pow(2, motor_index));
    }
  }

  //    std::cout << "[AFTER]the motor received id is: " << int(motor_index) <<
  //    std::endl;
  uint8_t data_length;
  buildVariable(data_length, byteStream);

  buildVariable(data_.motor_work_state[motor_index], byteStream);

  buildVariable(data_.motor_location[motor_index],
                byteStream);  // 2047为0度位置，单位0.00153398rad

  buildVariable(data_.motor_speed[motor_index],
                byteStream);  //单位0.00153398rad/s

  buildVariable(data_.motor_load[motor_index], byteStream);

  buildVariable(data_.motor_voltage[motor_index], byteStream);  //单位0.1V

  buildVariable(data_.motor_temp[motor_index], byteStream);

  uint8_t unused;
  for (int i = 0; i < 5; i++) {
    buildVariable(unused, byteStream);
  }
  buildVariable(data_.motor_current[motor_index - 1], byteStream);
  if (byteStream.size() == 0) {
    //ROS_ERROR("Motor status :  deserialise true 02");
    return true;
  } else {
    //ROS_ERROR("Motor status :  deserialise false 02");

    return false;
  }
}
}  // namespace xarm
