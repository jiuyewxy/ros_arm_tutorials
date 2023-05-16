#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
class TFPublish{
public:
  TFPublish(){
    ROS_INFO("tf_pub C++ demo.");
    // 创建/tf_pub/marker_pose话题的订阅端，回调处理函数为poseCB
    marker_pose_sub_ = nh_.subscribe("/tf_pub/marker_pose", 100, &TFPublish::poseCB, this);
    // 创建/tf话题的发布端
    pub_tf_ = nh_.advertise<tf2_msgs::TFMessage>("/tf",10);
    // 创建一个TransformStamped对象: base_link — head_link
    geometry_msgs::TransformStamped static_transformStamped;
    // 设置时间戳
    static_transformStamped.header.stamp = ros::Time::now();
    // 设置父坐标系
    static_transformStamped.header.frame_id = "base_link";
    // 设置子坐标系
    static_transformStamped.child_frame_id = "head_link";
    // 设置父坐标系到子坐标系的平移变换
    static_transformStamped.transform.translation.x = 0.1;
    static_transformStamped.transform.translation.y = 0.0;
    static_transformStamped.transform.translation.z = 0.3;
    // 设置父坐标系到子坐标系的旋转变换(四元数表示)
    tf2::Quaternion q;
    q.setRPY(0,0,0);
    static_transformStamped.transform.rotation.x = q.x();
    static_transformStamped.transform.rotation.y = q.y();
    static_transformStamped.transform.rotation.z = q.z();
    static_transformStamped.transform.rotation.w = q.w();
    // 创建一个静态TF发布对象
    static tf2_ros::StaticTransformBroadcaster static_br;
    // 广播 base_link — head_link 的静态TF
    static_br.sendTransform(static_transformStamped);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_tf_;
  ros::Subscriber marker_pose_sub_;

  // /marker_pose话题回调处理函数
  void poseCB(const geometry_msgs::PoseStamped &msg){
    //  创建TransformStamped对象dynamic_tf_1: world — base_link
    geometry_msgs::TransformStamped dynamic_tf_1;
    dynamic_tf_1.header.stamp = ros::Time::now();
    dynamic_tf_1.header.frame_id = "world";
    dynamic_tf_1.child_frame_id = "base_link";
    // 平移变换随Marker变化，base_link始终位于Marker下方
    dynamic_tf_1.transform.translation.x = msg.pose.position.x;
    dynamic_tf_1.transform.translation.y = msg.pose.position.y;
    dynamic_tf_1.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0,0,0);
    dynamic_tf_1.transform.rotation.x = q.x();
    dynamic_tf_1.transform.rotation.y = q.y();
    dynamic_tf_1.transform.rotation.z = q.z();
    dynamic_tf_1.transform.rotation.w = q.w();
    // 广播 world — base_link 的动态TF
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(dynamic_tf_1);

    // 创建TransformStamped对象dynamic_tf_2: head_link — camera_link
    // camera_link的俯仰角随Marker位置变化，Z轴始终指向Marker中心
    geometry_msgs::TransformStamped dynamic_tf_2;
    dynamic_tf_2.header.stamp = ros::Time::now();
    dynamic_tf_2.header.frame_id = "head_link";
    dynamic_tf_2.child_frame_id = "camera_link";
    dynamic_tf_2.transform.translation.x = 0;
    dynamic_tf_2.transform.translation.y = 0;
    dynamic_tf_2.transform.translation.z = 0.08;
    double pitch = std::atan((msg.pose.position.z - 0.08 - 0.3)/ 0.1) - 3.1415926/2.0;
    q.setRPY(0,pitch,0);
    dynamic_tf_2.transform.rotation.x = q.x();
    dynamic_tf_2.transform.rotation.y = q.y();
    dynamic_tf_2.transform.rotation.z = q.z();
    dynamic_tf_2.transform.rotation.w = q.w();
    // 发布 head_link — camera_link 的动态TF
    tf2_msgs::TFMessage tfm;
    tfm.transforms.push_back(dynamic_tf_2);
    pub_tf_.publish(tfm);
  }

};
int main(int argc, char **argv){
  ros::init(argc, argv, "tf_pub");
  TFPublish tf_pub_demo;
  ros::spin();
  return 0;
}
