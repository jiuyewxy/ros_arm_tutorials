#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
class  CvBridgeDemo{
public:
  CvBridgeDemo():nh_("~"){
    image_sub_ = nh_.subscribe("camera_image", 100, &CvBridgeDemo::imageCallBack, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image>("image_show", 10);
    ROS_INFO("cv_bridge_demo C++ demo is ready ......");
  }
  ~CvBridgeDemo(){}
private:
  ros::NodeHandle nh_;
  ros::Publisher image_pub_;
  ros::Subscriber image_sub_;
  void imageCallBack(const sensor_msgs::ImageConstPtr data){
    // 使用cv_bridge()将ROS图像转换成OpenCV格式
    cv_bridge::CvImagePtr cv_image_ptr;
    try {
      cv_image_ptr = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception e) {
      ROS_ERROR_STREAM("Cv_bridge Exception:"<<e.what());
      return;
    }
    // 获取图像像素的行数、列数
    int rows = cv_image_ptr->image.rows;
    int cols = cv_image_ptr->image.cols;

    // 判断图像是否有足够的区域用来画一个长方形，若有，则在图像上画一个橙色长方形
    if(rows>130 && cols>120){
      cv::rectangle(cv_image_ptr->image,cvPoint(30,30),cvPoint(90,100),cv::Scalar(0, 140, 255),-1 );
    }
    cv::imshow("Image window", cv_image_ptr->image);
    cv::waitKey(3);

    // 将画上长方形的图像转换回ROS图像消息格式，并发布到/cv_bridge_demo/image_show话题
    try {
      image_pub_.publish(cv_image_ptr->toImageMsg());
    } catch (cv_bridge::Exception e) {
      ROS_ERROR_STREAM("Cv_bridge Exception:"<<e.what());
      return;
    }
  }
};
int main(int argc, char **argv){
  ros::init(argc, argv, "cv_bridge_demo");
  CvBridgeDemo cv_bridge_demo;
  ros::spin();
  return 0;
}
