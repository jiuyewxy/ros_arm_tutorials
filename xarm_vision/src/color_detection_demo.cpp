#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include "xarm_vision/HSVColorDetectionConfig.h"

class ColorDetectionDemo{
public:
  ColorDetectionDemo():nh_("~"){
    nh_.param<int>("H_min", H_min_, 38);
    nh_.param<int>("H_max", H_max_, 57);
    nh_.param<int>("S_min", S_min_, 36);
    nh_.param<int>("S_max", S_max_, 208);
    nh_.param<int>("V_min", V_min_, 113);
    nh_.param<int>("V_max", V_max_, 255);
    image_sub_ = nh_.subscribe("camera_image", 100, &ColorDetectionDemo::imageCallBack, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image>("/color_test_result", 10);
    dyn_server_ = new dynamic_reconfigure::Server<xarm_vision::HSVColorDetectionConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<xarm_vision::HSVColorDetectionConfig>::CallbackType cb = boost::bind(&ColorDetectionDemo::dynamicReconfigureCallback, this, _1, _2);
    dyn_server_->setCallback(cb);
    ROS_INFO("color_detection_demo C++ demo is ready ......");
  }

private:
  ros::NodeHandle nh_;
  int H_min_,H_max_,S_min_,S_max_,V_min_,V_max_;
  dynamic_reconfigure::Server<xarm_vision::HSVColorDetectionConfig> *dyn_server_;
  ros::Publisher image_pub_;
  ros::Subscriber image_sub_;
  // 图像话题回调函数
  void imageCallBack(const sensor_msgs::ImageConstPtr data){
    // 将ROS图像消息转化成OpenCV图像
    cv_bridge::CvImagePtr cv_image_ptr;
    try {
      cv_image_ptr = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception e) {
      ROS_ERROR_STREAM("Cv_bridge Exception:"<<e.what());
    }
    // 颜色识别
    colorDetection(cv_image_ptr);
  }

  // 颜色识别与结果发布
  void colorDetection(cv_bridge::CvImagePtr cv_image_ptr){
    cv::Mat cv_image = cv_image_ptr->image;
    // 使用高斯滤波对原始图像进行减躁处理
    cv::Mat cv_image_blurred;
    cv::GaussianBlur(cv_image,cv_image_blurred, cv::Size(7, 7),0);
    // 将图像转换为HSV(Hue, Saturation, Value)模型
    cv::Mat cv_image_HSV;
    cv::cvtColor(cv_image_blurred,cv_image_HSV,cv::COLOR_BGR2HSV);
    // 颜色检测，得到目标颜色的二值图像
    cv::Mat mask;
    cv::inRange(cv_image_HSV,cv::Scalar(H_min_,S_min_,V_min_),cv::Scalar(H_max_,S_max_,V_max_),mask);
    // 对二值图像进行一些处理
    // 得到尺寸为(7, 7)的椭圆形元素
    cv::Mat kernal = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
    // 开操作，先腐蚀，再膨胀，可清除一些小东西(亮的)，放大局部低亮度的区域
    cv::morphologyEx(mask,mask,cv::MORPH_OPEN,kernal);
    // 闭操作，先膨胀，再腐蚀，可清除小黑点,连接一些区域
    cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,kernal);
    // 设置图像显示窗口Mask的大小和位置，将处理后的二值图像进行显示
    cv::namedWindow("Mask", cv::WINDOW_NORMAL);
    cv::moveWindow("Mask", 30,30);
    cv::imshow("Mask",mask);    cv::Mat mask_close;
    cv::waitKey(3);

    // 将原始图像和二值图像进行“与”操作，图像中除了识别出的颜色区域，其余全部"黑色"
    cv::Mat color_test_result;
    cv::bitwise_and(cv_image,cv_image,color_test_result,mask);
    // 设置图像显示窗口ColorTest的大小和位置，将"与"之后的图像进行显示
    cv::namedWindow("ColorTest",cv::WINDOW_NORMAL);
    cv::moveWindow("ColorTest",30,600);
    cv::imshow("ColorTest", color_test_result);
    cv::waitKey(3);

    // 对二值图像进行轮廓检测并绘制轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(cv_image,contours,-1,cv::Scalar(0,0,255),3);
    // 对轮廓列表中的每个轮廓，计算最小外接矩形并绘制矩形和矩形的中心
    cv::Point2f rect[4];
    for(auto c : contours){
      cv::RotatedRect box = cv::minAreaRect(c);
      cv::Rect bound_rect =  cv::boundingRect(c);
      // 绘制最小外接矩形的中心点
      cv::circle(cv_image,cv::Point(box.center.x,box.center.y),3,cv::Scalar(255, 0, 0),-1);
      // 绘制最小外接矩形
      box.points(rect);
      cv::rectangle(cv_image, bound_rect, cv::Scalar(255, 0, 0), 3);
    }
    // 将原图像上使用轮廓和矩形标注的识别结果图像转换回ROS图像消息格式，并发布到/color_test_result话题
    try {
      cv_image_ptr->image = cv_image;
      image_pub_.publish(cv_image_ptr->toImageMsg());
    } catch (cv_bridge::Exception e) {
      ROS_ERROR_STREAM("Cv_bridge Exception:"<<e.what());
    }

  }

  void dynamicReconfigureCallback(xarm_vision::HSVColorDetectionConfig &config, uint32_t level){
    H_min_ = config.H_min;
    H_max_ = config.H_max;
    S_min_ = config.S_min;
    S_max_ = config.S_max;
    V_min_ = config.V_min;
    V_max_ = config.V_max;
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "color_detection_demo");
  ColorDetectionDemo color_detection_demo;
  ros::spin();
  return 0;
}
