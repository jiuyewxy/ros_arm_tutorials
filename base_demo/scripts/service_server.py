#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from base_demo.srv import SetTargetDetec

class MinimalService(Node):

    def __init__(self):
        super().__init__('service_server')
        self.get_logger().info('service_server python node is Ready!')
        self.srv = self.create_service(SetTargetDetec, 'target_detection', self.target_detection_handle)

    def target_detection_handle(self, request, response):
        # 打印输出req.name
        self.get_logger().info('Target object is ' + request.name)
        self.get_logger().info('Find ' + request.name + ' and response.')
        # 对res的成员进行赋值
        response.success = True
        response.pose.position.x = 0.35
        response.pose.position.y = -0.35
        response.pose.position.z = 0.1
        response.pose.orientation.w = 1.0
        return response


def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()