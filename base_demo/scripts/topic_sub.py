#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from base_demo.msg import RobotInfo
# 接收到消息后,进入该回调函数
def callback(data):
    if data.pose.position.x == 20:
        rospy.loginfo('The robot has reached the target pose.')
        if data.is_carry:
            rospy.loginfo('The robot is carrying objects')
        else:
            rospy.loginfo('The robot is not carrying objects')
    else:
         rospy.loginfo(data.state)
         rospy.loginfo('Robot pose x : %.2fm; y : %.2fm; z : %.2fm', data.pose.position.x, data.pose.position.y, data.pose.position.z)

def listener():
    # 初始化节点
    rospy.init_node('topic_sub', anonymous=True)
    # 打印输出日志消息
    rospy.loginfo('topic_sub node is Ready!')
    # 创建/robot_info话题的订阅端,话题的回调处理函数为callback
    rospy.Subscriber('/robot_info', RobotInfo, callback)
    # 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    listener()
