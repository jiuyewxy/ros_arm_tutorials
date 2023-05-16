#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import dynamic_reconfigure.server
from xarm_vision.cfg import HSVColorDetectionConfig

class ColorDetectionDemo():

    def __init__(self):
        # 初始化节点
        rospy.init_node("color_detection_demo")
        # 设置参数
        self.H_min = rospy.get_param("~H_min", 38)
        self.H_max = rospy.get_param("~H_max", 57)
        self.S_min = rospy.get_param("~S_min", 36)
        self.S_max = rospy.get_param("~S_max", 208)
        self.V_min = rospy.get_param("~V_min", 113)
        self.V_max = rospy.get_param("~V_max", 255)
        # 创建动态配置服务器对象
        dyn_server = dynamic_reconfigure.server.Server(HSVColorDetectionConfig, self.dynamic_reconfigure_callback)

        # 创建cv_bridge对象
        self.bridge = CvBridge()

        # 创建话题/color_test_result的发布端，用来发布颜色识别结果
        self.image_pub = rospy.Publisher("/color_test_result",Image, queue_size=10)

        # 订阅的摄像头的彩色图像话题
        self.image_sub = rospy.Subscriber("~camera_image", Image, self.image_callback)
        rospy.loginfo("color_detection_demo Python demo is ready ......")
        rospy.spin()

    def image_callback(self, data):
        # 使用cv_bridge()将ROS图像转换成OpenCV格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)
        # 调用color_detection函数进行颜色识别
        self.color_detection(cv_image)

    # 颜色识别与结果发布函数
    def color_detection(self, cv_image):
        # 使用高斯滤波对原始图像进行减躁处理
        cv_image_blurred = cv2.GaussianBlur(cv_image, (7, 7), 0)

        # 将图像转换为HSV(Hue, Saturation, Value)模型
        cv_image_HSV = cv2.cvtColor(cv_image_blurred, cv2.COLOR_BGR2HSV)

        # 设置想要遮挡的区域的HSV值
        colorLow = np.array([self.H_min,self.S_min,self.V_min])
        colorHigh = np.array([self.H_max,self.S_max,self.V_max])

        # 颜色检测，得到目标颜色的二值图像
        mask = cv2.inRange(cv_image_HSV, colorLow, colorHigh)

        # 对二值图像进行一些处理
        # 得到尺寸为(7, 7)的椭圆形元素
        kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        # 开操作，去除一些噪点
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)
        # 闭操作，连接一些区域
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)


        # 设置图像显示窗口Mask的大小和位置，将处理后的二值图像进行显示
        cv2.namedWindow('Mask',cv2.WINDOW_NORMAL);
        cv2.moveWindow('Mask',30,30)
        cv2.imshow('Mask', mask)
        cv2.waitKey(3)

        # 将原始图像和二值图像进行“与”操作，图像中除了识别出的颜色区域，其余全部"黑色"
        color_test_result = cv2.bitwise_and(cv_image, cv_image, mask = mask)

        # 设置图像显示窗口ColorTest的大小和位置，将"与"之后的图像进行显示
        cv2.namedWindow('ColorTest',cv2.WINDOW_NORMAL);
        cv2.moveWindow('ColorTest',30,600)
        cv2.imshow('ColorTest', color_test_result)
        cv2.waitKey(3)

        # 对二值图像进行轮廓检测并绘制轮廓
        #不同版本opencv，findContours函数返回的参数个数不同
        _, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cv_image,contours,-1,(0,0,255),3)

        # 对轮廓列表中的每个轮廓，计算最小外接矩形并绘制矩形和矩形的中心
        for c in contours:
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)  # 将坐标规范化为整数
            cv2.drawContours(cv_image, [box], 0, (255, 0, 0), 3)
            cv2.circle(cv_image,(int(rect[0][0]),int(rect[0][1])),3,(255, 0, 0),-1)

        # 将原图像上使用轮廓和矩形标注的识别结果图像转换回ROS图像消息格式，并发布到/color_test_result话题
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def dynamic_reconfigure_callback(self, config, level):
        self.H_min = config.H_min
        self.H_max = config.H_max
        self.S_min = config.S_min
        self.S_max = config.S_max
        self.V_min = config.V_min
        self.V_max = config.V_max
        return config

if __name__ == '__main__':
    ColorDetectionDemo()
