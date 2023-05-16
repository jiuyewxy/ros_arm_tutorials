#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
from dynamic_reconfigure.server import Server
from advance_demo.cfg import PubMarkerConfig
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import math

class PubMarker():
    def  __init__(self):
        rospy.init_node('pub_marker')
        rospy.loginfo("pub_marker Python node is Ready!")
        # 获取参数的值
        self.user_name = rospy.get_param('~user_name', 'Xiaode')
        self.frame = rospy.get_param('~marker_frame', 'world')
        self.marker_size = rospy.get_param('~marker_size', 0.2)
        self.color_r = rospy.get_param('~marker_color_r', 1)
        self.color_g = rospy.get_param('~marker_color_g', 0)
        self.color_b = rospy.get_param('~marker_color_b', 0)
        self.color_a = rospy.get_param('~marker_color_a', 1)
        self.speed = rospy.get_param('~speed', 1.5)

        # 创建一个动态参数配置的服务端对象,回调函数为dynamic_reconfigure_callback
        dyn_server = Server(PubMarkerConfig, self.dynamic_reconfigure_callback)

        # 创建话题/marker_pose的发布端,话题的消息类型为geometry_msgs/PoseStamped
        pose_pub = rospy.Publisher('marker_pose', PoseStamped, queue_size=10)
        # 创建话题/target_marker的发布端,话题的消息类型为visualization_msgs/Marker
        marker_pub = rospy.Publisher('target_marker', Marker, queue_size=10)
        # 创建话题/user_name的发布端,话题的消息类型为visualization_msgs/Marker
        name_pub = rospy.Publisher('user_name', Marker, queue_size=10)

        # 创建Marker对象text_marker
        text_marker = Marker()
        # 设置text_marker的参考系为world
        text_marker.header.frame_id = "world"
        # 设置text_marker的类型为可视的有方向的文本TEXT_VIEW_FACING
        text_marker.type = Marker.TEXT_VIEW_FACING
        # 设置text_marker的ID
        text_marker.id = 1
        # 设置text_marker的行为为更改MODIFY
        text_marker.action = Marker.MODIFY
        # 设置text_marker的位姿pose
        text_marker.pose.position.z = 2
        text_marker.pose.orientation.w = 1.0
        # 设置text_marker的尺寸scale
        text_marker.scale.x = 0.5
        text_marker.scale.y = 0.5
        text_marker.scale.z = 0.5
        #  设置text_marker的颜色color
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0

        # 创建Marker对象marker
        marker = Marker()
        marker.id = 0   # id
        # marker类型为球体
        marker.type = Marker.SPHERE
        # marker行为为添加
        marker.action = Marker.ADD
        # marker在RViz中显示的时长，当Duration()无参数时，表示一直显示
        marker.lifetime = rospy.Duration() #
        # 创建位姿对象target_pose,用于Marker的位姿设置
        target_pose = PoseStamped()
        target_pose.pose.orientation.w = 1

        rate = 20
        theta = 0.0
        r = rospy.Rate(rate)
        # 以20Hz的频率循环向外发布marker,text_marker和marker的位姿
        while not rospy.is_shutdown():
            # marker在XY平面上做椭圆运动
            target_pose.header.frame_id = self.frame
            target_pose.pose.position.x = 1.0 + 0.5 * math.sin(theta)
            target_pose.pose.position.y = 0
            target_pose.pose.position.z = 0.8 + 0.3 * math.cos(theta)
            theta += self.speed / rate
            now = rospy.Time.now()
            target_pose.header.stamp = now
            marker.header.stamp = now
            marker.header.frame_id = target_pose.header.frame_id
            marker.pose.position = target_pose.pose.position
            marker.pose.orientation.w = 1
            # marker的尺寸颜色可通过动态参数动态设置
            marker.scale.x = self.marker_size
            marker.scale.y = self.marker_size
            marker.scale.z = self.marker_size
            marker.color.r = self.color_r
            marker.color.g = self.color_g
            marker.color.b = self.color_b
            marker.color.a = self.color_a
            text_marker.header.stamp = now
            # 设置text_marker的显示文本
            text_marker.text = "User name is : " + self.user_name
            # 发布话题消息
            pose_pub.publish(target_pose)
            marker_pub.publish(marker)
            name_pub.publish(text_marker)
            r.sleep()

    # 动态参数配置回调函数.当动态参数更新时,会调用回调函数进行处理
    def dynamic_reconfigure_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {user_name}, {marker_frame}, {marker_size}, {marker_color_r},\
                        {marker_color_g},{marker_color_b}, {marker_color_a}, {speed}""".format(**config))
        self.user_name = config.user_name
        self.frame = config.marker_frame
        self.marker_size = config.marker_size
        self.color_r = config.marker_color_r
        self.color_g = config.marker_color_g
        self.color_b = config.marker_color_b
        self.color_a = config.marker_color_a
        self.speed = config.speed
        return config

if __name__ == '__main__':
    marker = PubMarker()
    rospy.spin()
