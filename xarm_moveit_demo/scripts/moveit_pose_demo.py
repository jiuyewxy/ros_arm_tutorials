#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
from math import pi
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from  tf_conversions import transformations

class MoveItPoseDemo:
    def __init__(self):

        # 初始化Python API 依赖的moveit_commanderC++系统，需放在前面
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点，节点名为moveit_pose_demo
        rospy.init_node('moveit_pose_demo')

        # 把arm连接到规划组xarm
        arm = moveit_commander.MoveGroupCommander('xarm')

        planning_frame = arm.get_planning_frame()
        rospy.loginfo("Planning frame: %s", planning_frame)
        eef_link = arm.get_end_effector_link()
        rospy.loginfo("End effector link: %s", eef_link)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.02)
        arm.set_goal_orientation_tolerance(0.03)

        # 设置允许的最大速度
        arm.set_max_velocity_scaling_factor(0.8)

        # 设置末端执行器的目标位姿，参考坐标系为base_link
        target_pose = PoseStamped()
        target_pose.header.frame_id = planning_frame
        target_pose.header.stamp = rospy.Time.now()
        # 末端位置通过xyz设置
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.1
        target_pose.pose.position.z = 0.25
        # 末端姿态，四元数表示。通过quaternion_from_euler函数将RPY欧拉角转化为四元数
        quaternion = transformations.quaternion_from_euler(0,pi/2,0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
        rospy.loginfo("Moving to target_pose ...")

        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1)

        # 获取当前的位姿信息
        current_pose = arm.get_current_pose(eef_link)

        # 获取当前六个关节的位置信息
        current_joint_positions = arm.get_current_joint_values()

        # 末端执行器在base_link坐标系下，沿着x轴正方向移动5厘米
        rospy.loginfo("Move forward 5 cm ...")
        arm.shift_pose_target(0,0.05,eef_link)
        arm.go()
        rospy.sleep(1)

        # 控制机械臂回到初始化位置
        rospy.loginfo("Moving to pose: Home")
        arm.set_named_target('Home')
        arm.go()

        # 干净地关闭moveit_commander并退出程序
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItPoseDemo()
