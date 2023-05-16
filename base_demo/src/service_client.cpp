#include "ros/ros.h"
#include "base_demo/SetTargetDetec.h"

int main(int argc, char **argv){
  // 初始化ROS节点
  ros::init(argc, argv, "service_client");
  ROS_INFO("service_client node is Ready!");
  // 创建一个节点句柄（NodeHandle）对象nh
  ros::NodeHandle nh;
  // target_detection服务的服务端启动前，服务的调用一直处于阻塞状态
  ros::service::waitForService("target_detection");
  // 创建target_detection服务的客户端对象client
  ros::ServiceClient client = nh.serviceClient<base_demo::SetTargetDetec>("target_detection");
  // 定义服务类型的对象srv,为srv的请求数据成员赋值
  base_demo::SetTargetDetec srv;
  srv.request.name = "box";
  // 调用服务,若服务调用成功,call()返回true,服务的应答数据将保存到srv的response中
  if(client.call(srv)){
    if(srv.response.success){
      ROS_INFO("Target detection succeeded!");
      ROS_INFO_STREAM(srv.response.pose);
    }else{
      ROS_INFO("Can not find the target!");
    }
  }else{
    ROS_ERROR("Failed to call service target_detection");
  }
  return 0;
}
