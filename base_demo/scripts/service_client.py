#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from base_demo.srv import SetTargetDetec


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(SetTargetDetec, 'target_detection')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetTargetDetec.Request()

    def send_request(self):
        self.req.name = "box"
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    if response.success:
        minimal_client.get_logger().info('Target detection succeeded!')
        minimal_client.get_logger().info('Pose: x = %f ,y= %f, z= %f' 
        %(response.pose.position.x,response.pose.position.y,response.pose.position.z))
    else:
        minimal_client.get_logger().info('Can not find the target!')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



