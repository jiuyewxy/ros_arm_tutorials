#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import  Pose,  PoseStamped
from copy import deepcopy
import math
import numpy

class MoveItArclineDemo:
    def __init__(self):

        # 初始化Python API 依赖的moveit_commanderC++系统，需放在前面
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_arcline_demo', anonymous=True)

        # 连接到想要控制的规划组xarm
        arm = MoveGroupCommander('xarm')

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置允许的最大速度
        arm.set_max_velocity_scaling_factor(0.8)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)


        # 控制机械臂先回到初始化位置
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(1)

        # 设置圆弧中心为target_pose,先让机械臂运动到这个点
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.45
        target_pose.pose.orientation.w = 1
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1)

        # 初始化路径点列表
        waypoints = []

        # 在y-z平面内做圆弧运动,按照0.015弧度对圆进行切分，用一个个线段近似圆弧轨迹
        # y = 圆心的y坐标 + 半径×cos(th)，z = 圆心的z坐标 + 半径×sin(th)
        centerA = target_pose.pose.position.y
        centerB = target_pose.pose.position.z
        radius = 0.1
        # 把切分后的圆弧上的一系列路径点加入到waypoints中
        for th in numpy.arange(0, math.pi*2, 0.015):
            target_pose.pose.position.y = centerA + radius * math.cos(th)
            target_pose.pose.position.z = centerB + radius * math.sin(th)
            wpose = deepcopy(target_pose.pose)
            waypoints.append(deepcopy(wpose))

        # 设置规划相关参数
        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            # 规划路径 ，fraction返回1代表规划成功
            (plan, fraction) = arm.compute_cartesian_path (
                                            waypoints,   # waypoint poses，路点列表，这里是5个点
                                            0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
                                            0.0,         # jump_threshold，跳跃阈值，设置为0代表不允许跳跃
                                            True)        # avoid_collisions，避障规划
            # 尝试规划次数累加
            attempts += 1

            # 打印运动规划进程
            if attempts % 100 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # 如果路径规划成功（覆盖率100%），则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

        rospy.sleep(1)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(1)

        # 干净地关闭moveit_commander并退出程序
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItArclineDemo()
    except rospy.ROSInterruptException:
        pass
