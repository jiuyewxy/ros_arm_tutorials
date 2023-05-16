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

#ifndef PACKET_FINDER_H
#define PACKET_FINDER_H
#include <ecl/containers.hpp>
#include <iomanip>

namespace xarm {
/**
 * @brief
 * Provides simple packet finder which may be consist of stx, etx, payload, ...
 * @todo
 * (1) getting buffer from this class<br>
 * (2) simple construction of this class<br>
 * (3) device abstraction<br>
 * (4) extend-able packet structures<br>
 * (5) like ros, auto-generation of code from scripts<br>
 * (6) packetFinder to ros converter<br>
 * (7) evaluate it (valgrind)
 *
 */
class PacketFinderBase {
 public:
  typedef ecl::PushAndPop<unsigned char> BufferType;

  enum packetFinderState {
    clearBuffer = 0,
    waitingForStx,
    waitingForId,
    waitingForPayloadSize,
    waitingForPayloadToEtx,
    waitingForEtx,
  };
  enum packetFinderState state;

 protected:
  unsigned int size_stx;
  unsigned int size_etx;
  unsigned int size_length_field;
  bool variable_size_payload;
  unsigned int size_max_payload;
  unsigned int size_payload;
  unsigned int size_checksum_field;

  BufferType STX;
  BufferType ETX;
  BufferType buffer;

  bool verbose;

 public:
  PacketFinderBase(); /**< Default constructor. Use with configure(). **/

  virtual ~PacketFinderBase(){};

  void configure(const BufferType& putStx, const BufferType& putEtx,
                 unsigned int sizeLengthField, unsigned int sizeMaxPayload,
                 unsigned int sizeChecksumField, bool variableSizePayload);
  void clear();
  void enableVerbose();
  virtual bool update(const unsigned char* incoming,
                      unsigned int numberOfIncoming);
  virtual bool checkSum();
  unsigned int numberOfDataToRead();
  void getBuffer(BufferType& bufferRef);
  void getPayload(BufferType& bufferRef);

 protected:
  bool WaitForStx(const unsigned char datum);
  bool WaitForId(const unsigned char datum);
  bool waitForPayloadSize(const unsigned char datum);
  bool waitForEtx(const unsigned char incoming, bool& foundPacket);
  bool waitForPayloadAndEtx(const unsigned char* incoming,
                            unsigned int numberOfIncoming, bool& foundPacket);
};
}
#endif  // PACKET_FINDER_H
