#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "xarm_ros_wrapper.h"
int main(int argc, char **argv) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "xarm");
  // action名字必须与xarm_moveit_config/ros_controller.yaml里对应，所以这里nh没加xarm，后面重新生成配置文件后，再修改
  ros::NodeHandle nh("");
  std::string serial_port;
  int baud_rate;
    // 获取串口波特率
  nh.param<std::string>("/xarm/xarm_port", serial_port, "/dev/arm");

  nh.param<int>("/xarm/baud_rate", baud_rate, 1000000);
  xarm::XarmRosWrapper xarm_ros(nh,serial_port,baud_rate);
  ros::AsyncSpinner spinner(6);
  spinner.start();
  xarm_ros.halt();
  ros::waitForShutdown();

}
