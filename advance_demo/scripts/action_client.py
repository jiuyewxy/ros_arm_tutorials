#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from advance_demo.action import PickupPlace

class PickupPlaceActionClient(Node):

    def __init__(self):
        super().__init__('action_client')
        self._action_client = ActionClient(self, PickupPlace, 'pickup_place')

    def send_goal(self, order):
        goal_msg = PickupPlace.Goal()
        goal_msg.target_name = "box"
        goal_msg.target_pose.position.x = 0.35
        goal_msg.target_pose.position.y = -0.35
        goal_msg.target_pose.position.z = 0.1
        goal_msg.target_pose.orientation.w = 1.0

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Pickup and place the object successfully!')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Task completed %i%%' %feedback.percent_complete)


def main(args=None):
    rclpy.init(args=args)

    action_client = PickupPlaceActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()

