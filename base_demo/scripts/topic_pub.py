#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from base_demo.msg import RobotInfo


class TopicPublisher(Node):

    def __init__(self):
        super().__init__('topic_pub')
        self.get_logger().info('topic_pub python node is Ready!')
        self.publisher_ = self.create_publisher(RobotInfo, 'robot_info', 10)

        self.msg = RobotInfo()
        self.msg.is_carry = False
        self.msg.header.frame_id = 'map'
        self.msg.pose.position.x = 0.0
        self.msg.pose.position.y = 0.0
        self.msg.pose.position.z = 0.0
        self.msg.pose.orientation.w = 1.0
        self.go_flag = True
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        self.msg.state = 'Robot is moving...'
        if self.msg.pose.position.x == 0:
            self.go_flag = True
        if self.msg.pose.position.x == 20:
            self.go_flag = False
        if self.go_flag:
            self.msg.pose.position.x += 0.5
        else:
            self.msg.pose.position.x -= 0.5

        self.get_logger().info('Robot pose x : %.1fm' %self.msg.pose.position.x)
        self.publisher_.publish(self.msg)



def main(args=None):
    rclpy.init(args=args)
    topic_publisher = TopicPublisher()
    rclpy.spin(topic_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    topic_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





