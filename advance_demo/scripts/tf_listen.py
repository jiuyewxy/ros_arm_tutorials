#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped,PoseStamped
from std_msgs.msg import Empty
class TFListen():
    def __init__(self):
        rospy.init_node('tf_listen')
        rospy.loginfo("tf_listen Python demo")
        # 创建话题/set_car_follow_past的订阅端
        rospy.Subscriber('/set_car_follow_past', Empty, self.listen_past)
        # 监听world — base_link的最新TF变换
        self.listen_recent()

    # 监听world — base_link的最新TF变换
    def listen_recent(self):
        # 创建一个TF监听器对象listener,监听TF数据并存入tfBuffer
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        br = tf2_ros.TransformBroadcaster()
        # 以10Hz的频率循环获取 world到base_link 的变换
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                # 查询最新的 world - base_link的TF
                trans = tfBuffer.lookup_transform('world', 'base_link',rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            # 平移变换x乘以2,平移变换y乘以2,作为 world—car_1 的平移变换的值
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "car_1"
            t.transform = trans.transform
            t.transform.translation.x *=2
            t.transform.translation.y *=2
            # 广播 world—car_1 的动态TF
            br.sendTransform(t)
            rate.sleep()

    # /set_car_follow_past话题回调函数
    def listen_past(self,msg):
        # 创建一个TF监听器对象listener,监听TF数据并存入tfBuffer
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        br = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                # 查询5s前 world - base_link 的TF
                past = rospy.Time.now() - rospy.Duration(5.0)
                trans_past = tfBuffer.lookup_transform('world', 'base_link', past, rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            # 将trans_past作为 world—car_2 的平移变换的值,让car_2始终跟随base_link 5s前的位置
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "car_2"
            t.transform = trans_past.transform
            # 广播 world—car_2 的动态TF
            br.sendTransform(t)
            rate.sleep()

if __name__ == '__main__':
    try:
        tf_listen = TFListen()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TF Listener is shut down.")
