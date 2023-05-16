#ifndef XARM_DRIVER_H_
#define XARM_DRIVER_H_

#include <mutex>
#include <iostream>
#include <vector>
#include <math.h>
#include <chrono>
#include <vector>
#include "xarm_realtime_communication.h"

namespace xarm {

class XarmDriver {
private:
  std::vector<std::string> joint_names_;
  bool executing_traj_;

public:
  XarmRealtimeCommunication* xarm_interface_;
  XarmDriver(
      std::string serial_port_,uint32_t baud_rate_);
  ~XarmDriver(){
    delete xarm_interface_;

  }

  bool start();
  void halt();

  // 同步控制所有舵机的目标位置
  void sycnControl(std::vector<double> locations_rad, std::vector<double> velocitys_m);
  void singleJointControl(int motor_id, double joint_value);

  XarmData getArmMotorData();

  bool doTraj(std::vector<double> inp_timestamps,
      std::vector<std::vector<double> > inp_positions,
      std::vector<std::vector<double> > inp_velocities);

  void getRobotPos();

  void stopTraj();

  std::vector<double> interpCubic(double t, double T,
      std::vector<double> p0_pos, std::vector<double> p1_pos,
      std::vector<double> p0_vel, std::vector<double> p1_vel);

  std::vector<std::string> getJointNames();
  void setJointNames(std::vector<std::string> jn);

  void setRobotIO(int type, int mode,int index,float state);

  void setServojTime(double t);


  /************************Aubo plan and move API*****************************/
  void initMoveProfile();
  void setBlock(bool flag);
  void setMaxSpeed(double speed);
  void setMaxAcc(double acc);
  void movej(std::vector<double> positions);
  void movel(std::vector<double> positions);
  void movelTo(std::vector<double> positions);
  void addWayPoint(std::vector<double> positions);
  void movep(double blendRadius,int trackMode);

};

}



#endif /* AUBO_NEW_DRIVER_H_ */
