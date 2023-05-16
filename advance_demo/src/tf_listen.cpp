#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Empty.h>
#include <thread>

class TFListen{
public:
  TFListen(tf2_ros::Buffer& tfBuffer):tf_buffer_(tfBuffer){
    ROS_INFO("tf_listen C++ demo");
    // 创建话题/set_car_follow_past的订阅端
    listen_past_ = nh_.subscribe("/set_car_follow_past", 100, &TFListen::listenPastCB, this);
    // 启动新线程用于监听world — base_link的最新TF变换
    std::thread listen_recent_thread(&TFListen::listenRecent,this);
    listen_recent_thread.detach();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber listen_past_;
  tf2_ros::Buffer& tf_buffer_;

  // 监听world — base_link的最新TF变换
  void listenRecent(){
    //tf2_ros::Buffer tf_buffer;
    //tf2_ros::TransformListener listener(tf_buffer);

    static tf2_ros::TransformBroadcaster br;

    // 以10Hz的频率循环获取 world - base_link 的最新TF变换
    ros::Rate rate(10.0);
    while(nh_.ok()){
      geometry_msgs::TransformStamped trans;
      try{
        //trans = tfBuffer.lookupTransform("world", "base_link",ros::Time(0),ros::Duration(3.0));
        // 查询最新的 world - base_link的TF
        trans = tf_buffer_.lookupTransform("world", "base_link",ros::Time(0));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        rate.sleep();
        continue;
      }
      // 平移变换x乘以2,平移变换y乘以2,作为 world—car_1 的平移变换的值
      geometry_msgs::TransformStamped t;
      t.header.stamp = ros::Time::now();
      t.header.frame_id = "world";
      t.child_frame_id = "car_1";
      t.transform = trans.transform;
      t.transform.translation.x *=2;
      t.transform.translation.y *=2;
      // 广播 world—car_1 的动态TF
      br.sendTransform(t);
      rate.sleep();
    }
  }
  // /set_car_follow_past话题回调函数
  void listenPastCB(const std_msgs::EmptyConstPtr){
    ROS_INFO("Start listen past...");
    //tf2_ros::Buffer tf_buffer;
    //tf2_ros::TransformListener listener(tf_buffer);
    static tf2_ros::TransformBroadcaster br;
    ros::Rate rate(10.0);
    while(nh_.ok()){
      geometry_msgs::TransformStamped trans_past;
      try{
        // 监听world — base_link 5s前的TF变换trans_past
        ros::Time past = ros::Time::now() - ros::Duration(5.0);
        trans_past = tf_buffer_.lookupTransform("world", "base_link", past, ros::Duration(3.0));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        rate.sleep();
        continue;
      }
      // 将trans_past作为 world—car_2 的平移变换的值,让car_2始终跟随base_link 5s前的位置
      geometry_msgs::TransformStamped t;
      t.header.stamp = ros::Time::now();
      t.header.frame_id = "world";
      t.child_frame_id = "car_2";
      t.transform = trans_past.transform;
      // 广播 world—car_2 的动态TF
      br.sendTransform(t);
      rate.sleep();
    }

  }
};
int main(int argc, char **argv){
  ros::init(argc, argv, "tf_listen");
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener listener(tfBuffer);
  TFListen tf_listen(tfBuffer);
  ros::spin();
  return 0;
}
