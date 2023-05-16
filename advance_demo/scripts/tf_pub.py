#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from  tf_conversions import transformations
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped,PoseStamped
import math
class TFPublish():
    def __init__(self):
        rospy.init_node('tf_pub')
        # 创建/tf_pub/marker_pose话题的订阅端，回调处理函数为pose_cb
        rospy.Subscriber('/tf_pub/marker_pose',PoseStamped,self.pose_cb)
        # 创建/tf话题的发布端
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1)
        rospy.loginfo("tf_pub python demo.")
        # 创建一个TransformStamped对象: base_link — head_link
        static_transformStamped = TransformStamped()
        # 设置时间戳
        static_transformStamped.header.stamp = rospy.Time.now()
        # 设置父坐标系
        static_transformStamped.header.frame_id = "base_link"
        # 设置子坐标系
        static_transformStamped.child_frame_id = "head_link"
        # 设置父坐标系到子坐标系的平移变换
        static_transformStamped.transform.translation.x = 0.1
        static_transformStamped.transform.translation.y = 0.
        static_transformStamped.transform.translation.z = 0.3
        # 设置父坐标系到子坐标系的旋转变换(四元数表示)
        q = transformations.quaternion_from_euler(0,0,0)
        static_transformStamped.transform.rotation.x = q[0]
        static_transformStamped.transform.rotation.y = q[1]
        static_transformStamped.transform.rotation.z = q[2]
        static_transformStamped.transform.rotation.w = q[3]
        # 创建一个静态TF发布对象
        static_br = tf2_ros.StaticTransformBroadcaster()
        # 广播 base_link — head_link 的静态TF
        static_br.sendTransform(static_transformStamped)

    # /marker_pose回调函数
    def pose_cb(self, msg):
        # 创建TransformStamped对象dynamic_tf_1: world — base_link
        dynamic_tf_1 = TransformStamped()
        dynamic_tf_1.header.stamp = rospy.Time.now()
        dynamic_tf_1.header.frame_id = "world"
        dynamic_tf_1.child_frame_id = "base_link"
        # 平移变换随Marker变化，base_link始终位于Marker下方
        dynamic_tf_1.transform.translation.x = msg.pose.position.x
        dynamic_tf_1.transform.translation.y = msg.pose.position.y
        dynamic_tf_1.transform.translation.z = 0.0
        q1 = transformations.quaternion_from_euler(0, 0, 0)
        dynamic_tf_1.transform.rotation.x = q1[0]
        dynamic_tf_1.transform.rotation.y = q1[1]
        dynamic_tf_1.transform.rotation.z = q1[2]
        dynamic_tf_1.transform.rotation.w = q1[3]
        # 广播 world — base_link 的动态TF
        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(dynamic_tf_1)

        # 创建TransformStamped对象dynamic_tf_2: head_link — camera_link
        # camera_link的俯仰角随Marker位置变化，Z轴始终指向Marker中心
        dynamic_tf_2 = TransformStamped()
        dynamic_tf_2.header.stamp = rospy.Time.now()
        dynamic_tf_2.header.frame_id = "head_link"
        dynamic_tf_2.child_frame_id = "camera_link"
        dynamic_tf_2.transform.translation.x = 0
        dynamic_tf_2.transform.translation.y = 0
        dynamic_tf_2.transform.translation.z = 0.08
        pitch = math.atan((msg.pose.position.z - 0.08 - 0.3)/0.1) -3.1415926/2
        q2 = transformations.quaternion_from_euler(0, pitch, 0)
        dynamic_tf_2.transform.rotation.x = q2[0]
        dynamic_tf_2.transform.rotation.y = q2[1]
        dynamic_tf_2.transform.rotation.z = q2[2]
        dynamic_tf_2.transform.rotation.w = q2[3]
        # 发布 head_link — camera_link 的动态TF
        tfm = TFMessage([dynamic_tf_2])
        self.pub_tf.publish(tfm)


if __name__ == '__main__':
    try:
        tf_pub = TFPublish()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TF publisher is shut down.")
