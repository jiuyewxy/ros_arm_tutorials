#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import rclpy.node

def param_demo():
    rclpy.init()
    node = rclpy.node.Node("param_demo")
    # 设置参数
    node.declare_parameter('/a_string', 'hello word')
    node.declare_parameter('list_of_floats', [1., 2.1, 3.2, 4.3])
    node.declare_parameter('~private_int', 2)
    node.declare_parameter('bool_True', "true")
    node.declare_parameter('gains', "{'p': 1, 'i': 2, 'd': 3}")
    
    # 获取参数的值
    string_param = node.get_parameter('/a_string').get_parameter_value().string_value
    node.get_logger().info('Param /a_string : %s' %string_param)

    string_param = node.get_parameter('~private_int').get_parameter_value().integer_value
    node.get_logger().info('Param ~private_int : %s' %string_param)


if __name__ == '__main__':
    param_demo()

