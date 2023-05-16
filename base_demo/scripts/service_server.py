#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from base_demo.srv import SetTargetDetec, SetTargetDetecResponse
# 服务的回调处理函数,req为服务的请求
def target_detection_handle(req):
    # 打印输出req.name
    rospy.loginfo('Target object is ' + req.name)
    rospy.loginfo('Find ' + req.name + ' and response.')
    # 定义应答SetTargetDetecResponse的对象res
    res = SetTargetDetecResponse()
    # 对res的成员进行赋值
    res.success = True
    res.pose.position.x = 0.35
    res.pose.position.y = -0.35
    res.pose.position.z = 0.1
    res.pose.orientation.w = 1
    # return语句返回应答
    return res

def server():
    # 初始化节点
    rospy.init_node('service_server')
    # 创建target_detection服务的服务端server,服务类型SetTargetDetec,服务回调函数target_detection_handle
    server = rospy.Service('target_detection', SetTargetDetec, target_detection_handle)
    rospy.loginfo('Service server is Ready!')
    rospy.spin()

if __name__ == '__main__':
    server()
