#include "ros/ros.h"
int main(int argc, char **argv){
  // 初始化ROS节点
  ros::init(argc, argv, "param_demo");
  // 创建一个节点句柄（NodeHandle）对象nh
  ros::NodeHandle nh("~");
  // 设置参数
  nh.setParam("/a_string", "hello word");
  std::vector<double> vector_param = {1.0, 2.1, 3.2, 4.3};
  nh.setParam("/vector_of_double", vector_param);
  nh.setParam("relative_int", 2);
  ros::param::set("bool_True", true);
  std::map<std::string,int> gains = {{"p", 1}, {"i", 2},{ "d", 3}};
  ros::param::set("gains", gains);
  // 获取当前所有参数的名字
  std::vector<std::string> param_names;
  nh.getParamNames(param_names);
  ros::param::getParamNames(param_names);
  ROS_INFO_STREAM("Param names: ");
  for(auto it : param_names){
    ROS_INFO_STREAM(it);
  }
  // 获取参数的值
  std::string string_param;
  nh.getParam("/a_string",string_param);
  ROS_INFO_STREAM("Param /a_string : "<<string_param);
  std::vector<double> vector_param_value;
  nh.getParam("/vector_of_double",vector_param_value);
  ROS_INFO_STREAM("Param /vector_of_double : [ ");
  for(auto it : vector_param_value){
    ROS_INFO_STREAM(it);
  }
  ROS_INFO_STREAM("]");
  std::map<std::string,int> gains_value;
  ros::param::get("gains",gains_value);
  ROS_INFO("p: %i, i: %i, d: %i", gains_value["p"] ,gains_value["i"] , gains_value["d"]);
  // 从最近的命名空间查找参数并返回参数的完整名字
  std::string  param_name;
  if(nh.searchParam("relative_int",param_name)){
    int int_param;
    ros::param::get(param_name,int_param);
    ROS_INFO_STREAM("Param "<< param_name<<" : "<<int_param);
  }
  // 获取参数的值,若参数服务器中没有该参数，则使用默认值
  int default_param;
  ros::param::param<int>("default_param", default_param, 100);
  ROS_INFO("default_param: %i",default_param);
  // 删除参数
  if(nh.deleteParam("a_string")){
    ROS_INFO("Successfully deleted param /a_string ");
  }
  if(ros::param::del("vector_of_double")){
    ROS_INFO("Successfully deleted param vector_of_double");
  }
  // 判断参数是否存在
  if (nh.hasParam("/bool_True")){
    ROS_INFO("/bool_True exits.");
  }else{
    ROS_ERROR("/bool_True does not exit.");
  }
  if (ros::param::has("default_param")){
    ROS_INFO("default_param exits.");
  }else{
    ROS_ERROR("default_param does not exit.");
  }
  return 0;
}
