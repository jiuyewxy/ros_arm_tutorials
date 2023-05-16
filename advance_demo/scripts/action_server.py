#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from advance_demo.msg import PickupPlaceAction, PickupPlaceFeedback, PickupPlaceResult
class PickupPlaceServer():
    def __init__(self):
        # 创建action服务端对象
        self.server =  actionlib.SimpleActionServer('pickup_place', PickupPlaceAction, self.executeCB, auto_start = False)
        # 启动action的服务端
        self.server.start()
        rospy.loginfo('action_server is Ready!')

    # 服务端接收到action的goal时的回调执行函数
    def executeCB(self, goal):
        # 打印输出goal中的target_name和target_pose
        rospy.loginfo('The target object is ' + goal.target_name)
        rospy.loginfo(goal.target_pose)
        rospy.loginfo('Start to pickup and place...')
        # 创建feedback用来记录执行过程中的反馈信息
        feedback = PickupPlaceFeedback()
        # 创建result用来记录执行完的结果
        result = PickupPlaceResult()
        ## 通过循环,模拟任务完成的进度百分比
        success = True
        r = rospy.Rate(1)
        for i in range(10):
            # 若有抢占请求（目标被取消、收到新的目标),当前目标的状态设置为抢占（PREEMPTED）
            if self.server.is_preempt_requested():
                rospy.loginfo('/pickup_place: Preempted')
                self.server.set_preempted()
                success = False
                break
            feedback.percent_complete += 10
            # 发布目标的执行过程反馈feedback
            self.server.publish_feedback(feedback)
            r.sleep()
        # 若目标任务执行成功,发送目标执行结果
        if success:
            rospy.loginfo('/pickup_place: Succeeded')
            result.success = True
            self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('action_server')
    action_server = PickupPlaceServer()
    rospy.spin()
