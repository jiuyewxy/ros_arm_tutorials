#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from advance_demo.action import PickupPlace
import time

class PickupPlaceServer(Node):
    def __init__(self):
        super().__init__('action_server')
        # 创建action服务端对象
        self._action_server = ActionServer(
            self,
            PickupPlace,
            'pickup_place',
            self.execute_callback)
        # 启动action的服务端
        self.get_logger().info('action_server is Ready!')

    # 服务端接收到action的goal时的回调执行函数
    def execute_callback(self, goal_handle):
        # 打印输出goal中的target_name和target_pose
        self.get_logger().info('The target object is ' + goal_handle.request.target_name)
        self.get_logger().info('Pose: x = %f ,y= %f, z= %f' 
        %(goal_handle.request.target_pose.position.x,goal_handle.request.target_pose.position.y,goal_handle.request.target_pose.position.z))
        self.get_logger().info('Start to pickup and place...')
        # 创建feedback用来记录执行过程中的反馈信息
        feedback = PickupPlace.Feedback()
        # 创建result用来记录执行完的结果
        result = PickupPlace.Result()
        ## 通过循环,模拟任务完成的进度百分比
        for i in range(10):
            feedback.percent_complete += 10
            self.get_logger().info('Task completed %i%%' %feedback.percent_complete)
            # 发布目标的执行过程反馈feedback
            goal_handle.publish_feedback(feedback) 
            time.sleep(1)
        goal_handle.succeed()
        self.get_logger().info('/pickup_place: Succeeded')
        result.success = True
        return result # 返回动作结果

def main(args=None):
    rclpy.init(args=args)
    pickup_place_action_server = PickupPlaceServer()
    rclpy.spin(pickup_place_action_server)
    pickup_place_action_server.destory_node() # 销毁节点对
    rclpy.shutdown() # 关闭ros2 python接口


if __name__ == '__main__':
    main()

