#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from base_demo.srv import *

def client():
    # 初始化节点
    rospy.init_node('service_client')
    rospy.loginfo('service_client node is Ready!')
    # 在target_detection服务的服务端启动前,服务的调用一直处于阻塞状态
    rospy.wait_for_service("target_detection")
    # 创建对象client用来调用target_detection服务
    client = rospy.ServiceProxy('target_detection', SetTargetDetec)
    try:
        # 服务调用的其他形式
        #req = SetTargetDetecRequest()
        #req.name = 'box'
        #res = client(req)
        #res = client(name='box')

        # 进行服务调用,res保存服务端返回的应答数据
        res = client('box')
        if res.success:
            rospy.loginfo('Target detection succeeded!')
            rospy.loginfo(res.pose)
        else:
            rospy.loginfo('Can not find the target!')
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

if __name__ == '__main__':
    client()
