#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from base_demo.msg import RobotInfo

def talker():
    # 初始化节点
    rospy.init_node('topic_pub', anonymous=False)
    # 打印输出日志消息
    rospy.loginfo('topic_pub node is Ready!')
    # 创建发布RobotInfo消息到话题/robot_info的句柄(发布端)pub
    pub = rospy.Publisher('/robot_info', RobotInfo, queue_size=10)
    # 创建了RobotInfo消息的对象msg,并赋值
    msg = RobotInfo()
    msg.is_carry = False
    msg.header.frame_id = 'map'
    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.position.z = 0
    msg.pose.orientation.w = 1
    # 创建rate对象,设置频率为5Hz,用于循环发布
    rate = rospy.Rate(5)
    # 节点关闭前一直循环发布消息
    while not rospy.is_shutdown():
        # 设置state并改变msg中机器人在X方向上的位置
        msg.state = 'Robot is moving...'
        if msg.pose.position.x == 0:
            go_flag = True
        if msg.pose.position.x == 20:
            go_flag = False
        if go_flag:
            msg.pose.position.x += 0.5
        else:
            msg.pose.position.x -= 0.5
        # header的时间戳为当前时间
        msg.header.stamp = rospy.Time.now()
        # 打印输出机器人X方向的位置
        rospy.loginfo('Robot pose x : %.1fm', msg.pose.position.x)
        # 发布消息
        pub.publish(msg)
        # 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    talker()





