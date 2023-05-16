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

#ifndef COMMAND_H
#define COMMAND_H
#include <ecl/containers.hpp>
#include "packet_handler/payload_base.hpp"
#include <serial/serial.h>
namespace xarm {

class Command : public packet_handler::PayloadBase {
 public:
  typedef ecl::PushAndPop<unsigned char> Buffer;
  typedef ecl::Stencil<Buffer> BufferStencil;

  /**
   * These values are used to detect the type of sub-payload that is ensuing.
   */
  enum Name {
    Read = 2,
    Write = 3,
    RegWrite = 4,
    Action = 5,
    Reset = 6,
    SycnWrite = 0x83
  };

  /**
   * @brief Data structure containing data for commands.
   *
   * It is important to keep this
   * state as it will have to retain knowledge of the last known command in
   * some instances - e.g. for gp_out commands, quite often the incoming command
   * is only to set the output for a single led while keeping the rest of the
   * current
   * gp_out values as is.
   *
   * For generating individual commands we modify the data here, then copy the
   * command
   * class (avoid doing mutexes) and spin it off for sending down to the device.
   */
  struct Data {
    Data()
        : command(Read),
          index(0),
          location(2047),
          read_start(0x38),
          read_length(15) {
      for (int i = 0; i < 6; i++) {
        sycn_location[i] = 2047;
        sycn_speed[i] = 0;
      }
    }

    Name command;
    uint8_t index;
    uint16_t location;
    uint8_t read_start;
    uint8_t read_length;
    uint16_t sycn_location[6];
    uint16_t sycn_speed[6];
  };

  virtual ~Command() {}

  //  读取某个舵机的ERROR信息
  static Command setSingleMotorErrorRead(const uint8_t &index);

  //  控制某个舵机运动
  static Command setSingleMotorControl(const uint8_t &index,
                                       const uint16_t &location);

  //  同步控制6个舵机运动
  static Command setSycnMotorControl(const std::vector<uint16_t> sycn_location,
                                     const std::vector<uint16_t> sycn_speed);

  //  读取某个舵机的状态信息
  static Command setSingleMotorStatusRead(const uint8_t &index);

  Data data;

  void resetBuffer(Buffer &buffer);
  bool serialise(ecl::PushAndPop<unsigned char> &byteStream, std::string arm_name);
  bool deserialise(ecl::PushAndPop<unsigned char> &byteStream, XarmData &data_,
                   uint16_t& received_motor) {
    return true;
  }



 private:
  static const unsigned char header0;
  static const unsigned char header1;
  std::string serial_port;
};

}  // namespace xbot

#endif  // COMMAND_H
