#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from advance_demo.msg import *
import actionlib

# done_cb在目标执行完时被调用一次
def done_cb(status, result):
    if status == 2:
        rospy.logerr("Program interrupted before completion")
    if result.success:
        rospy.loginfo('Pickup and place the object successfully!')
# active_cb在目标状态转换为ACTIVE时被调用一次
def active_cb():
    rospy.loginfo("Goal just went active")

# feedback_cb在每次接收到服务端发送的feedback时被调用
def feedback_cb(feedback):
    rospy.loginfo('Task completed %i%%', feedback.percent_complete)

def action_client():
    rospy.init_node('action_client')
    # 创建action客户端对象并与action的服务端连接
    client = actionlib.SimpleActionClient('pickup_place', PickupPlaceAction)
    rospy.loginfo("Waiting for action server to start.")
    # 等待action的服务端开启，若服务端没有开启,将一直等待
    client.wait_for_server()
    rospy.loginfo("Action server started, sending goal.")
    # 创建一个PickupPlaceGoal的对象,并对goal的成员进行赋值
    goal = PickupPlaceGoal()
    goal.target_name = "box"
    goal.target_pose.position.x = 0.35
    goal.target_pose.position.y = -0.35
    goal.target_pose.position.z = 0.1
    goal.target_pose.orientation.w = 1
    # 向action的服务端发送目标goal,并设置回调函数
    client.send_goal(goal, done_cb, active_cb, feedback_cb)
    # 阻塞,直到这个目标完成
    client.wait_for_result()

if __name__=="__main__":
    action_client()

