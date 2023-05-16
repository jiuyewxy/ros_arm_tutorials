#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import math
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp,PlaceLocation,GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
GRIPPER_JOINT_NAMES = ['gripper_1_joint','gripper_2_joint']
GRIPPER_OPEN = [0.65,0.65]
GRIPPER_GRASP = [0.1,0.1]
BASE_LINK = 'base_link'
TABLE_ID = "table"
TARGET_ID = "target"

class MoveItPickPlaceDemo:
    def __init__(self):
        # 初始化Python API 依赖的moveit_commanderC++系统
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化节点
        rospy.init_node('moveit_pick_place_demo')
        rospy.loginfo("Pick and Place demo is ready. ")

        # 初始化场景对象
        self.scene = PlanningSceneInterface()
        rospy.sleep(1)
        # 在规划场景中添加物体
        self.add_collision_object()

        # 初始化规划组
        self.xarm_group = MoveGroupCommander("xarm")
        self.gripper_group = MoveGroupCommander("gripper")
        # 设置最大速度限制
        self.xarm_group.set_max_velocity_scaling_factor(0.8)
        self.gripper_group.set_max_velocity_scaling_factor(0.8)

        # 设置桌子table为抓取和放置操作的支撑面，使MoveIt!忽略物体放到桌子上时产生的碰撞警告
        self.xarm_group.set_support_surface_name(TABLE_ID)
        # 设置抓取列表
        grasps = self.make_grasps();
        # 设置result标记每次抓取尝试的结果
        max_pick_attempts = 7
        result = None
        n_attempts = 0
        rospy.loginfo("Try to pick up the box. ")
        # 循环尝试抓取操作直到抓取成功或最大尝试次数用尽
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = self.xarm_group.pick(TARGET_ID, grasps)
            rospy.sleep(0.2)

        # 设置一个放置目标位姿
        place_pose = PoseStamped()
        place_pose.header.frame_id = BASE_LINK
        place_pose.pose.position.x = 0.32
        place_pose.pose.position.y = -0.32
        place_pose.pose.position.z = 0.22 / 2.0
        q = quaternion_from_euler(0, 0, -math.pi/4)
        place_pose.pose.orientation.x = q[0]
        place_pose.pose.orientation.y = q[1]
        place_pose.pose.orientation.z = q[2]
        place_pose.pose.orientation.w = q[3]
        # 生成一系列放置位姿
        places = self.make_places(place_pose)
        # 如果抓取成功，尝试物品放置操作。
        if result == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Picked up successfully. Try to place the box. ")
            # 定义放置操作的最大尝试次数
            max_place_attempts = 7
            # 重置标记操作结果的result和尝试次数
            result = None
            n_attempts = 0

            # 循环放置直到放置成功或超过最大尝试次数
            while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
                n_attempts += 1
                rospy.loginfo("Place attempt: " +  str(n_attempts))
                for place in places:
                    result = self.xarm_group.place(TARGET_ID, place)
                    if result == MoveItErrorCodes.SUCCESS:
                        break
                rospy.sleep(0.2)

        rospy.loginfo("Move to Home. ")
        # 回到初始位姿
        self.xarm_group.set_named_target('Home')
        self.xarm_group.go()
        # 闭合手爪
        self.gripper_group.set_named_target('Close_gripper')
        self.gripper_group.go()
        rospy.sleep(1)
        # 删除规划场景里物体
        rospy.loginfo("Remove objects. ")
        self.scene.remove_world_object(TABLE_ID)
        self.scene.remove_world_object(TARGET_ID)
        rospy.sleep(1)
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    #
    def add_collision_object(self):
        # 设置桌子的长宽高 [l, w, h]
        table_size = [1.0, 1.2, 0.01]
        # 设置桌子的位姿
        table_pose = PoseStamped()
        table_pose.header.frame_id = BASE_LINK
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = - table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        # 把桌子添加到规划场景中
        self.scene.add_box(TABLE_ID, table_pose, table_size)

        # 设置目标物体的长宽高 [l, w, h]
        target_size = [0.05, 0.05, 0.22]
        # 设置目标的位姿，让目标物体位于桌子上
        target_pose = PoseStamped()
        target_pose.header.frame_id = BASE_LINK
        target_pose.pose.position.x = 0.47
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z =  target_size[2] / 2.0
        target_pose.pose.orientation.w = 1.0
        # 把目标物体添加到规划场景中
        self.scene.add_box(TARGET_ID, target_pose, target_size)
        rospy.sleep(1)

    # 生成一系列可能的抓取列表
    def make_grasps(self):
        # 初始化抓取列表
        grasps = []
        # 把变量grasp初始化为Grasp消息类型的对象
        grasp = Grasp()
        # 设置抓取的位姿grasp_pose
        grasp.grasp_pose.header.frame_id = BASE_LINK
        grasp.grasp_pose.pose.position.x = 0.47
        grasp.grasp_pose.pose.position.y = 0
        grasp.grasp_pose.pose.position.z = 0.14
        q = quaternion_from_euler(0, 0, 0)
        grasp.grasp_pose.pose.orientation.x = q[0]
        grasp.grasp_pose.pose.orientation.y = q[1]
        grasp.grasp_pose.pose.orientation.x = q[2]
        grasp.grasp_pose.pose.orientation.w = q[3]
        # 设置pre_grasp_approach,沿着X轴正向靠近抓取点,移动的最小距离为0.1m，期望距离为0.12米
        grasp.pre_grasp_approach.direction.header.frame_id = BASE_LINK
        grasp.pre_grasp_approach.direction.vector.x = 1.0
        grasp.pre_grasp_approach.min_distance = 0.1
        grasp.pre_grasp_approach.desired_distance = 0.12
        # 设置post_grasp_retreat,抓取物体后，沿着Z轴正向撤离，移动的距离最小为0.08米，期望距离为0.1米
        grasp.post_grasp_retreat.direction.header.frame_id = BASE_LINK
        grasp.post_grasp_retreat.direction.vector.z = 1.0;
        grasp.post_grasp_retreat.min_distance = 0.08;
        grasp.post_grasp_retreat.desired_distance = 0.1;
        # 设置夹爪在抓取物品前的位姿为张开的状态
        grasp.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN);
        # 设置夹爪用于抓取对象时的位置
        grasp.grasp_posture = self.make_gripper_posture(GRIPPER_GRASP);
        # 添加到grasps列表
        grasps.append(deepcopy(grasp))
        return grasps

    # 通过手爪关节的位置生成JointTrajectory消息类型的轨迹返回值
    def make_gripper_posture(self, joint_positions):
        posture = JointTrajectory()
        # 设置joint_names为gripper规划组的两个关节名
        posture.joint_names = GRIPPER_JOINT_NAMES
        tp = JointTrajectoryPoint()
        # 设置关节的位置
        tp.positions = joint_positions
        tp.time_from_start = rospy.Duration(0.5)
        posture.points.append(tp)
        return posture

    # 生成一系列可能的放置点位姿
    def make_places(self, init_pose):
        # 创建moveit_msgs/PlaceLocation消息的对象place
        place = PlaceLocation()
        # 设置放置位姿
        place.place_pose = init_pose
        # 设置靠近放置点的方向、最小移动距离和期望距离，这里设置为沿着Z轴向下移动0.1米
        place.pre_place_approach.direction.header.frame_id = BASE_LINK
        place.pre_place_approach.direction.vector.z = -1.0
        place.pre_place_approach.min_distance = 0.08
        place.pre_place_approach.desired_distance = 0.1
        # 设置放置完成后机械臂的撤离方向、移动最小距离和期望距离，这里设置为沿着Z轴向上移动0.15米
        place.post_place_retreat.direction.header.frame_id = BASE_LINK
        place.post_place_retreat.direction.vector.z = 1.0
        place.post_place_retreat.min_distance = 0.12
        place.post_place_retreat.desired_distance = 0.15

        # 可尝试的x位置偏移量
        x_vals = [0, 0.005, 0.01, -0.005, -0.01]
        # 初始化放置位姿列表
        places = []
        # 在放置位置附近生成其他可放置位置，并添加到放置位姿列表
        for x in x_vals:
            place.place_pose.pose.position.x = init_pose.pose.position.x + x
            places.append(deepcopy(place))

        # 返回放置列表
        return places

if __name__ == "__main__":
    MoveItPickPlaceDemo()
