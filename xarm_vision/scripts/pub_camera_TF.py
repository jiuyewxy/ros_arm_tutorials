#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import geometry_msgs.msg
from easy_handeye.handeye_calibration import HandeyeCalibration
import dynamic_reconfigure.server
import threading

from xarm_vision.cfg import SetTFOffsetConfig

class PubBaseCameraTF():

    def __init__(self):
        # 初始化节点
        rospy.init_node("pub_camera_TF")
        # 设置参数
        self.X_offset = rospy.get_param("~X_offset", 0)
        self.Y_offset = rospy.get_param("~Y_offset", 0)
        self.Z_offset = rospy.get_param("~Z_offset", 0)

        # 创建动态配置服务器对象
        dyn_server = dynamic_reconfigure.server.Server(SetTFOffsetConfig, self.dynamic_reconfigure_callback)

#        while rospy.get_time() == 0.0:
#            pass
        inverse = rospy.get_param('inverse')

        self.calib = HandeyeCalibration.from_file(rospy.get_namespace())

        if self.calib.parameters.eye_on_hand:
            overriding_robot_effector_frame = rospy.get_param('robot_effector_frame')
            if overriding_robot_effector_frame != "":
                self.calib.transformation.header.frame_id = overriding_robot_effector_frame
        else:
            overriding_robot_base_frame = rospy.get_param('robot_base_frame')
            if overriding_robot_base_frame != "":
                self.calib.transformation.header.frame_id = overriding_robot_base_frame
        overriding_tracking_base_frame = rospy.get_param('tracking_base_frame')
        if overriding_tracking_base_frame != "":
            self.calib.transformation.child_frame_id = overriding_tracking_base_frame

        rospy.loginfo('loading calibration parameters into namespace {}'.format(
            rospy.get_namespace()))
        HandeyeCalibration.store_to_parameter_server(self.calib)
        self.init_x = self.calib.transformation.transform.translation.x
        self.init_y = self.calib.transformation.transform.translation.y
        self.init_z = self.calib.transformation.transform.translation.z

        self.t1 = threading.Thread(target=self.pub_TF)
        self.t1.setDaemon(True)
        self.t1.start()

        rospy.spin()
    def pub_TF(self):
        orig = self.calib.transformation.header.frame_id  # tool or base link
        dest = self.calib.transformation.child_frame_id  # tracking_base_frame

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.transform = self.calib.transformation.transform

        static_transformStamped.header.frame_id = orig
        static_transformStamped.child_frame_id = dest
        print self.calib.transformation.transform
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.transform.translation.x = self.init_x +self.X_offset
            static_transformStamped.transform.translation.y = self.init_y +self.Y_offset
            static_transformStamped.transform.translation.z = self.init_z +self.Z_offset
#            print static_transformStamped.transform.translation.x
#            print self.X_offset
            broadcaster.sendTransform(static_transformStamped)
            rate.sleep()


    def dynamic_reconfigure_callback(self, config, level):
        self.X_offset = config.X_offset
        self.Y_offset = config.Y_offset
        self.Z_offset = config.Z_offset
        return config


if __name__ == '__main__':
    PubBaseCameraTF()


