#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "advance_demo/PickupPlaceAction.h"

class PickupPlaceServer{
public:
  PickupPlaceServer():
    // 初始化action服务端as_
    as_(nh_, "pickup_place", boost::bind(&PickupPlaceServer::goalCB, this, _1),false)
  {
    // 启动action的服务端
    as_.start();
    ROS_INFO("action_server is Ready!");
  }
  ~PickupPlaceServer(){}

  // 服务端接收到action的goal时的回调执行函数
  void goalCB(const advance_demo::PickupPlaceGoalConstPtr goal){
    // 创建feedback用来记录执行过程中的反馈信息
    advance_demo::PickupPlaceFeedback feedback;
    // 创建result用来记录执行完的结果
    advance_demo::PickupPlaceResult result;
    // 打印输出goal中的target_name和target_pose
    ROS_INFO_STREAM("The target object is " << goal->target_name);
    ROS_INFO_STREAM(goal->target_pose);
    ROS_INFO("Start to pickup and place...");
    // 通过循环,模拟任务完成的进度百分比
    bool success = true;
    ros::Rate r(1);
    for(int i = 0; i<10; i++){
      // 若有抢占请求（目标被取消、收到新的目标),当前目标的状态设置为抢占（PREEMPTED）
      if(as_.isPreemptRequested() || !ros::ok()){
        ROS_INFO("/pickup_place: Preempted");
        as_.setPreempted();
        success = false;
        break;
      }
      feedback.percent_complete += 10;
      // 发布目标的执行过程反馈feedback
      as_.publishFeedback(feedback);
      r.sleep();
    }

    // 若目标任务执行成功,发送目标执行结果
    if (success){
      ROS_INFO("/pickup_place: Succeeded");
      result.success = true;
      as_.setSucceeded(result);
    }
  }
private:
  ros::NodeHandle nh_;
  // 声明服务端SimpleActionServer的对象as_
  actionlib::SimpleActionServer<advance_demo::PickupPlaceAction> as_;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "action_server");
  PickupPlaceServer action_server;
  ros::spin();
  return 0;
}
