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

#ifndef PACKET_BASE_HPP
#define PACKET_BASE_HPP
#include <stdint.h>
#include <ecl/containers.hpp>
#include "../data_types.h"
namespace packet_handler {
class PayloadBase {
 public:
  /**
   * this is simple magic to write the flag, when we get the packet from the
   * host or
   * when we want to send the data
   */
  bool yes;

  /**
   * it indicates the type of derived packet. if packet type is dynamic, length
   * of
   * packet can be changed during communication session. Ohterwise can not.
   */
  const bool is_dynamic;

  /**
   * it indicates length of data part of packet, except header and length field.
   * if packet is fixed type, this value should be matched with length field.
   * if packet is dynamic type, this value indicates minimal value of length
   * field.
   */
  const unsigned char length;

  /*
   * construct and destruct
   */
  PayloadBase(const bool is_dynamic_ = false, const unsigned char length_ = 0)
      : yes(false), is_dynamic(is_dynamic_), length(length_){};
  virtual ~PayloadBase(){};
  /*
   * serialisation
   */
  virtual bool serialise(ecl::PushAndPop<unsigned char>& byteStream, std::string arm_name) = 0;
  virtual bool deserialise(ecl::PushAndPop<unsigned char>& byteStream,
                           xarm::XarmData& data_, uint16_t& received_motor) = 0;

  // utilities
  // todo; let's put more useful converters here. Or we may use generic
  // converters
 protected:
  // below funciton should be replaced wiht converter
  //###########################################################
  //#数据在计算机内存中存储是高位在前，低位在后，int型10存储为：00000000000000000000000000001010
  //#数据在串口传输过程中是低位在前，高位在后,因而10存储为00001010000000000000000000000000
  //###########################################################
  template <typename T>
  void buildVariable(T& V, ecl::PushAndPop<unsigned char>& buffer) {
    if (buffer.size() < sizeof(T)) return;
    V = static_cast<unsigned char>(buffer.pop_front());

    unsigned int size_value(sizeof(T));
    for (unsigned int i = 1; i < size_value; i++) {
      V |= ((static_cast<unsigned char>(buffer.pop_front())) << (8 * i));
    }
  }

  template <typename T>
  void buildBytes(const T& V, ecl::PushAndPop<unsigned char>& buffer) {
    unsigned int size_value(sizeof(T));
    for (unsigned int i = 0; i < size_value; i++) {
      buffer.push_back(static_cast<unsigned char>((V >> (i * 8)) & 0xff));
    }
  }
};

/**
 * Need to be very careful with this - it will only work across platforms if
 * they
 * happen to be doing reinterpret_cast with the same float standard.
 * @param V
 * @param buffer
 */
template <>
inline void PayloadBase::buildVariable<float>(
    float& V, ecl::PushAndPop<unsigned char>& buffer) {
  if (buffer.size() < 4) return;
  unsigned int ui;
  ui = static_cast<unsigned char>(buffer.pop_front());

  unsigned int size_value(4);
  for (unsigned int i = 1; i < size_value; i++) {
    ui |= ((static_cast<unsigned char>(buffer.pop_front())) << (8 * i));
  }

  V = reinterpret_cast<float&>(ui);
}

template <>
inline void PayloadBase::buildBytes<float>(
    const float& V, ecl::PushAndPop<unsigned char>& buffer) {
  if (buffer.size() < 4) return;
  unsigned int size_value(4);
  unsigned int ui(reinterpret_cast<const unsigned int&>(V));
  for (unsigned int i = 0; i < size_value; i++) {
    buffer.push_back(static_cast<unsigned char>((ui >> (i * 8)) & 0xff));
  }
}
};

#endif  // PACKET_BASE_HPP
