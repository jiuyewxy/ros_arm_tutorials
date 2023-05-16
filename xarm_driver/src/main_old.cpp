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

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "xarm_ros.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "xarm");
  // action名字必须与xarm_moveit_config/ros_controller.yaml里对应，所以这里nh没加xarm，后面重新生成配置文件后，再修改
  ros::NodeHandle nh("");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);
  xarm::XarmRos xarm(nh, buffer);
  xarm.start();
  ros::AsyncSpinner spinner(6);
  spinner.start();

  ros::waitForShutdown();
   // 为了测试代码，我把跟真机相关的部分代码都注释掉了。一般都以/**/注释的。
/*  xarm.startUpdate();*/
  //  cpp ros node program must include ros::spin(), or subscribe and publisher
  //  will not exacute
  //ros::spin();
  //ros::shutdown();
}
