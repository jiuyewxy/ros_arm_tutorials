#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, Pose

class MoveItBeelineDemo:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_beeline_demo', anonymous=True)

        # 初始化需要控制的规划组
        arm = MoveGroupCommander('xarm')

        # 允许重新规划
        arm.allow_replanning(True)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.005)
        arm.set_goal_orientation_tolerance(0.005)

        # 设置允许的最大速度
        arm.set_max_velocity_scaling_factor(0.8)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(1)

        # 设置三角形第一个顶点的位姿，让机械臂从初始状态运动到这个点
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.w = 1

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose)

        # 规划并执行
        traj = arm.go()
        rospy.sleep(1)

        # 获取当前位置并保存到start_pose里，可以用来作为路径点的起点和终点
        start_pose = arm.get_current_pose().pose
        end_pose = deepcopy(start_pose)

        # 初始化路点列表
        waypoints = []

        # 设置第一个路径点为start_pose
        wpose = deepcopy(start_pose)

        # 将start_pose加入路点列表
        waypoints.append(start_pose)

        # 设置第二个路径点, 把第二个路径点加入路点列表
        wpose.position.z -= 0.2
        waypoints.append(deepcopy(wpose))

        # 设置第三个路径点,把第三个路径点加入路点列表
        wpose.position.y += 0.2
        waypoints.append(deepcopy(wpose))

        # 把第四个路径点（end_pose,与第一个路径点重合）加入路点列表
        waypoints.append(deepcopy(end_pose))

        fraction = 0.0   # 路径规划覆盖率
        maxtries = 100   # 最大尝试规划次数
        attempts = 0     # 已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表，这里是4个点
                                    0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
                                    0.0,         # jump_threshold，跳跃阈值，设置为0代表不允许跳跃
                                    True)        # avoid_collisions，避障规划
            # 尝试次数累加
            attempts += 1
            # 打印运动规划进程
            if attempts % 100 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
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

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItBeelineDemo()
    except rospy.ROSInterruptException:
        pass
