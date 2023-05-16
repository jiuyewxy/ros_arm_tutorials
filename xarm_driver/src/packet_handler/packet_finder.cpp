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

#include "packet_handler/packet_finder.h"
#include <ros/ros.h>
#include <fstream>
#include <sstream>
using namespace std;
namespace xarm {
PacketFinderBase::PacketFinderBase() : state(waitingForStx), verbose(false) {}

void PacketFinderBase::configure(const BufferType &putStx,
                                 const BufferType &putEtx,
                                 unsigned int sizeLengthField,
                                 unsigned int sizeMaxPayload,
                                 unsigned int sizeChecksumField,
                                 bool variableSizePayload) {
  size_stx = putStx.size();
  size_etx = putEtx.size();
  size_length_field = sizeLengthField;
  variable_size_payload = variableSizePayload;
  size_max_payload = sizeMaxPayload;
  size_payload = variable_size_payload ? 0 : sizeMaxPayload;
  size_checksum_field = sizeChecksumField;
  STX = putStx;
  ETX = putEtx;
  buffer = BufferType(size_stx + size_length_field + size_max_payload +
                      size_checksum_field + size_etx);
  state = waitingForStx;
  clear();
}

void PacketFinderBase::clear() {
  state = waitingForStx;
  buffer.clear();
}

void PacketFinderBase::enableVerbose() { verbose = true; }

bool PacketFinderBase::checkSum() { return true; }

unsigned int PacketFinderBase::numberOfDataToRead() {
  unsigned int num(0);

  switch (state) {
    case waitingForEtx:
      num = 1;
      break;

    case waitingForPayloadToEtx:
      num = size_payload;
      break;

    case waitingForPayloadSize:
    case waitingForStx:
    case waitingForId:
    case clearBuffer:
    default:
      num = 1;
      break;
  }

  if (verbose) {
    printf("[state(%d):%02d]", state, num);
  }
  return num;
}

void PacketFinderBase::getBuffer(BufferType &bufferRef) {
//  ROS_ERROR("Packer finder :  getBuffer 01");
  bufferRef = buffer;

//  ROS_ERROR("Packer finder :  getBuffer 02");
}

void PacketFinderBase::getPayload(BufferType &bufferRef) {
//  ROS_ERROR("Packer finder :  getPayload 01");

  bufferRef.clear();

  bufferRef.resize(buffer.size() - size_stx - size_checksum_field);
  for (unsigned int i = size_stx; i < buffer.size() - size_checksum_field;
       i++) {
    bufferRef.push_back(buffer[i]);
  }
//  ROS_ERROR("Packer finder :  getPayload 02");

  //  std::cout<<"buffer.size:"<<buffer.size()<<std::endl;

  //  ofstream outfile("~/debug.txt", ofstream::app);
  //  outfile<<"buffer size:"<<buffer.size()<<endl<<"stx size:"<<size_stx;
}

/**
 * Checks for incoming packets.
 *
 * @param incoming
 * @param numberOfIncoming
 * @return bool : true if a valid incoming packet has been found.
 */
bool PacketFinderBase::update(const unsigned char *incoming,
                              unsigned int numberOfIncoming) {
  // clearBuffer = 0, waitingForStx, waitingForPayloadSize,
  // waitingForPayloadToEtx, waitingForEtx,
  // std::cout << "update [" << numberOfIncoming << "][" << state << "]" <<
  // std::endl;
//  ROS_ERROR("Packer finder :  update 01");
  if (!(numberOfIncoming > 0)) return false;

  bool found_packet(false);

  if (state == clearBuffer) {
    buffer.clear();
    state = waitingForStx;
  }
  switch (state) {
    case waitingForStx:
      if (WaitForStx(incoming[0])) {
        state = waitingForId;
      }
      break;
    case waitingForId:
      if (WaitForId(incoming[0])) {
        if (size_length_field) {
          state = waitingForPayloadSize;
        } else {
          if (variable_size_payload) {
            // e.g. stargazer
            state = waitingForEtx;
          } else {
            // e.g. iroboQ
            // Todo; should put correct state
            state = waitingForPayloadToEtx;
          }
        }
      }
      else {
        state=clearBuffer;
      }

      break;
    case waitingForEtx:
      if (waitForEtx(incoming[0], found_packet)) {
        state = clearBuffer;
      }
      break;

    case waitingForPayloadSize:
      if (waitForPayloadSize(incoming[0])) {
        state = waitingForPayloadToEtx;
      }
      break;

    case waitingForPayloadToEtx:
      if (waitForPayloadAndEtx(incoming, numberOfIncoming, found_packet)) {
        state = clearBuffer;
      }
      break;

    default:
      state = waitingForStx;
      break;
  }
  if (found_packet) {
//    ROS_ERROR("Packer finder :  update 02");

    return checkSum();
  } else {
//    ROS_ERROR("Packer finder :  update  false 02 ");

    return false;
  }
}
/*****************************************************************************
** Protected
*****************************************************************************/

bool PacketFinderBase::WaitForStx(const unsigned char datum) {
  bool found_stx(true);

  // add incoming datum
  buffer.push_back(datum);

  // check whether we have STX
  for (unsigned int i = 0; i < buffer.size() && i < STX.size(); i++) {
    if (buffer[i] != STX[i]) {
      found_stx = false;
      buffer.pop_front();
      break;
    }
  }

  return (found_stx && buffer.size() == STX.size());
}

bool PacketFinderBase::WaitForId(const unsigned char datum) {
  bool found_id(true);
  int id = datum;

  if(id>14) {
//   ROS_ERROR("the id is: %d",id);
    return false;}
  buffer.push_back(datum);

  return found_id;
}

bool PacketFinderBase::waitForPayloadSize(const unsigned char datum) {
  // push data
  buffer.push_back(datum);

  size_payload = datum;
//  std::cout<<size_payload<<std::endl;
  return true;
}

bool PacketFinderBase::waitForEtx(const unsigned char incoming,
                                  bool &foundPacket) {
  // push data
  buffer.push_back(incoming);

  // check when we need to wait for etx
  // if minimum payload size is 1
  if (buffer.size() < size_stx + size_etx + 1) {
    return false;
  } else {
    unsigned int number_of_match(0);
    for (unsigned int i = 0; i < ETX.size(); i++) {
      if (buffer[buffer.size() - ETX.size() + i] == ETX[i]) {
        number_of_match++;
      }
    }

    if (number_of_match == ETX.size()) {
      foundPacket = true;
      return true;
    }

    if (buffer.size() >= size_stx + size_max_payload + size_etx)
      return true;
    else
      return false;
  }
}

bool PacketFinderBase::waitForPayloadAndEtx(const unsigned char *incoming,
                                            unsigned int numberOfIncoming,
                                            bool &foundPacket) {
  // push data
  for (unsigned int i = 0; i < numberOfIncoming; i++) {
    buffer.push_back(incoming[i]);
  }
  /*********************
  ** Error Handling
  **********************/
  if (size_payload > size_max_payload) {
    state = clearBuffer;
    return false;
  }
  // check when we need to wait for etx
  if (buffer.size() < size_stx + size_length_field + size_payload +
                          size_checksum_field + size_etx) {
    return false;
  } else {
    //    std::cout<<buffer.size()<<std::endl;
    foundPacket = true;
    return true;
  }
}
}  // namespace xarm
