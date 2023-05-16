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

#ifndef DATA_TYPES_H
#define DATA_TYPES_H
#include <stdint.h>
#include <iostream>
#include <vector>
namespace xarm {
struct XarmData {
  std::vector<uint16_t> motor_location;
  std::vector<uint16_t> motor_speed;
  std::vector<uint16_t> motor_load;
  std::vector<uint8_t>  motor_voltage;
  std::vector<uint8_t> motor_temp;
  std::vector<uint8_t> motor_current;
  std::vector<uint8_t> motor_work_state;
//  uint16_t motor_location[7];
//  uint16_t motor_speed[7];
//  uint16_t motor_load[7];
//  uint8_t motor_voltage[7];
//  uint8_t motor_temp[7];
//  uint8_t motor_current[7];
//  uint8_t motor_work_state[7];
};
}

#endif  // DATA_TYPES_H
