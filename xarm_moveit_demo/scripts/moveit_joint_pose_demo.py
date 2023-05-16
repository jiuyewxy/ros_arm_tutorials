#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander

def MoveitJointPoseDemo():
    # 初始化Python API 依赖的moveit_commanderC++系统，需放在前面
    moveit_commander.roscpp_initialize(sys.argv)

    # 初始化ROS节点，节点名为'moveit_joint_pose_demo'
    rospy.init_node('moveit_joint_pose_demo', anonymous=True)

    # 连接到规划组。
    arm = moveit_commander.MoveGroupCommander('xarm')
    gripper = moveit_commander.MoveGroupCommander('gripper')

    # 设置机械臂和夹爪的关节运动允许误差，单位：弧度
    arm.set_goal_joint_tolerance(0.01)
    gripper.set_goal_joint_tolerance(0.01)

    # 设置允许的最大速度
    arm.set_max_velocity_scaling_factor(0.8)
    gripper.set_max_velocity_scaling_factor(0.8)

    rospy.loginfo("Moveing to joint-space goal: joint_positions")

    # 设置机械臂的目标位置，使用xarm组六个关节的位置数据进行描述（单位：弧度）
    joint_positions = [-0.664, -0.775, 0.675, -1.241, -0.473, -1.281]
    arm.set_joint_value_target(joint_positions)

    # 规划出一条从当前位姿到目标位姿的轨迹，存放在traj变量里
    #traj = arm.plan()
    plan_success,traj,planning_time,error_code=arm.plan()


    # 执行规划好的轨迹
    arm.execute(traj)
    rospy.sleep(1)

    rospy.loginfo("Open gripper ...")

    # 设置手爪规划组gripper里两个关节的位置为0.65，单位弧度
    joint_positions = [0.65, 0.65]
    gripper.set_joint_value_target(joint_positions)

    # 规划并执行，手爪张开
    gripper.go()
    rospy.sleep(1)
    rospy.loginfo("Close gripper ...")

    # 设置手爪规划组gripper的目标为Close_gripper,手爪闭合
    gripper.set_named_target('Close_gripper')
    gripper.go()
    rospy.sleep(1)
    rospy.loginfo("Moving to pose: Home")

    # 设置xarm目标位姿为Home，规划并执行
    arm.set_named_target('Home')
    arm.go()
    rospy.sleep(1)

    # 干净地关闭moveit_commander并退出程序
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveitJointPoseDemo()
