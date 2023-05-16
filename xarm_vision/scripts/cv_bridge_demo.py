#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class cvBridgeDemo():
    def __init__(self):
        rospy.init_node("cv_bridge_demo")
        # 创建cv_bridge对象
        self.bridge = CvBridge()
        # 订阅图像话题
        self.image_sub = rospy.Subscriber("~camera_image", Image, self.image_callback)
        # 创建图像话题的发布端，用来发布将OpenCV图像转化成sensor_msgs/Image的图像
        self.image_pub = rospy.Publisher("~image_show",Image, queue_size=10)
        rospy.loginfo("cv_bridge_demo Python demo is ready ......")

    def image_callback(self, data):
        # 使用cv_bridge()将ROS图像转换成OpenCV格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)
        # 获取图像像素的行数、列数和通道数
        (rows,cols,channels) = cv_image.shape
        # 判断图像是否有足够的区域用来画一个长方形，若有，则在图像上画一个橙色长方形
        if cols > 120 and rows > 130:
            # 圆边界限的粗细，单位像素，若等于-1表示以指定的颜色填充整个长方形
            thickness = -1
            # 在图像上画一个橙色长方形
            cv2.rectangle(cv_image, (30,30) ,(90,100) ,(0, 140, 255),thickness)
        # 打开一个窗口显示图像，
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        # 将画上长方形的图像转换回ROS图像消息格式，并发布到/cv_bridge_demo/image_show话题
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down cv_bridge_demo node.")
        cv2.DestroyAllWindows()
