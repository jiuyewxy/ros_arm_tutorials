#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from base_demo.msg import RobotInfo


class TopicSubscriber(Node):

    def __init__(self):
        super().__init__('topic_subscriber')
        self.get_logger().info('topic_sub python node is Ready!')

        self.subscription = self.create_subscription(
            RobotInfo,
            'robot_info',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, data):
        if data.pose.position.x == 20:
            self.get_logger().info('The robot has reached the target pose.')
            if data.is_carry:
                self.get_logger().info('The robot is carrying objects')
            else:
                self.get_logger().info('The robot is not carrying objects')
        else:
            self.get_logger().info(data.state)
            self.get_logger().info('Robot pose x : %.2fm' %data.pose.position.x )



def main(args=None):
    rclpy.init(args=args)

    topic_subscriber = TopicSubscriber()

    rclpy.spin(topic_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    topic_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

