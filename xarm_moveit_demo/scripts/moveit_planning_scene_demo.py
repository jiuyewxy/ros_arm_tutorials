#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys, os
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose
from math import pi
from copy import deepcopy
import tf
from tf.transformations import *

class MoveItObstaclesDemo:
    def __init__(self):

        # 初始化Python API 依赖的moveit_commanderC++系统，需放在前面
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点，节点名为'moveit_obstacles_demo'
        rospy.init_node('moveit_planning_scene_demo', anonymous=True)

        # 初始化场景对象
        scene = PlanningSceneInterface()
        rospy.sleep(1)

        # 初始化需要控制的规划组。手臂xarm，手爪gripper
        arm = moveit_commander.MoveGroupCommander('xarm')
        gripper = moveit_commander.MoveGroupCommander('gripper')

        # 设置允许的最大速度
        arm.set_max_velocity_scaling_factor(0.8)
        gripper.set_max_velocity_scaling_factor(0.8)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置目标位姿为Home
        arm.set_named_target('Home')
        arm.go()

        print "============ Press `Enter` to add objects to the planning scene ..."
        raw_input()
        # 将桌子(长方体桌面)添加到规划场景中
        table_id = 'table'
        table_size = [1.0, 1.2, 0.01]
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'base_link'
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z =  -table_size[2]/2
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
        # 判断是否已经将table添加到规划场景中
        if self.wait_for_state_update(table_id,scene,obstacle_is_known=True):
            rospy.loginfo("The Table has been successfully added.")
        else:
            rospy.loginfo("Failed to add the Table.")

        # 将一个长方体box添加到规划场景中
        # 设置长方体box1的ID、尺寸和位姿
        box_id = 'box'
        box_size = [0.25, 0.3, 0.04]
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'base_link'
        box_pose.pose.position.x = 0.4
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = box_size[2]/2 + 0.37
        box_pose.pose.orientation.w = 1.0
        # 调用add_box()函数将box添加到规划场景中
        scene.add_box(box_id, box_pose, box_size)

        # 判断是否已经将box添加到规划场景中
        if self.wait_for_state_update(box_id,scene,obstacle_is_known=True):
            rospy.loginfo("The Box has been successfully added.")
        else:
            rospy.loginfo("Failed to add the Box")

        # 将一个圆柱体添加到规划场景中
        cylinder_object = CollisionObject()
        cylinder_object.header.frame_id = "base_link"
        cylinder_object.id = "cylinder"
        cylinder_primitive = SolidPrimitive()
        cylinder_primitive.type = cylinder_primitive.CYLINDER
        cylinder_primitive.dimensions = [0.12, 0.015]
        cylinder_pose = Pose()
        cylinder_pose.position.x = 0.4
        cylinder_pose.position.y = 0
        cylinder_pose.position.z = cylinder_primitive.dimensions[0]/2.0 + box_size[2]/2 + 0.37
        cylinder_pose.orientation.w = 1.0
        cylinder_object.primitives =[cylinder_primitive]
        cylinder_object.primitive_poses =[cylinder_pose]
        cylinder_object.operation = cylinder_object.ADD
        scene.add_object(cylinder_object)
        if self.wait_for_state_update(cylinder_object.id,scene,obstacle_is_known=True):
            rospy.loginfo("The Cylinder has been successfully added.")
        else:
            rospy.loginfo("Failed to add the Cylinder.")

        print " "
        print "============ Press `Enter` to open gripper, move the arm and close gripper..."
        raw_input()
        # 手爪张开
        joint_positions = [0.65, 0.65]
        gripper.set_joint_value_target(joint_positions)
        gripper.go()
        rospy.sleep(1)
        # 设置机械臂末端执行器的目标位姿，参考坐标系为base_link
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.41
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = cylinder_primitive.dimensions[0]/2.0 + box_size[2]/2 + 0.37
        target_pose.pose.orientation.w = 1
        arm.set_start_state_to_current_state()
        # 设置目标target_pose并让机械臂运动到此目标
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1)
        # 闭合手爪
        joint_positions = [0.2, 0.2]
        gripper.set_joint_value_target(joint_positions)
        gripper.go()
        rospy.sleep(1)

        print " "
        print "============ Press `Enter` to attach the Cylinder to the XBot-Arm robot ..."
        raw_input()
        # 使用attach_object()函数把cylinder附着到机械臂末端执行器上
        attach_object = AttachedCollisionObject()
        attach_object.link_name = "gripper_centor_link"
        attach_object.object = cylinder_object
        scene.attach_object(attach_object)
        # 判断cylinder是否附着成功
        if self.wait_for_state_update(cylinder_object.id,scene,object_is_attached=True):
            rospy.loginfo("The cylinder has been successfully attached.")
        else:
            rospy.loginfo("Failed to attach the cylinder.")

        print " "
        print "============ Press `Enter` to move the arm to a position 25 cm down  ..."
        raw_input()
        arm.shift_pose_target(2,-0.25, "gripper_centor_link")
        arm.go()
        rospy.sleep(1)

        print " "
        print "============ Press `Enter` to detach the Cylinder from the XBot-Arm robot ..."
        raw_input()

        # 使用remove_attached_object()函数将cylinde从机械臂上分离
        scene.remove_attached_object("gripper_centor_link", cylinder_object.id)

        rospy.sleep(1)

        print " "
        print "============ Press `Enter` to remove the obstacles ..."
        raw_input()
        # 从规划场景里移除桌面、长方体和圆柱体
        scene.remove_world_object(table_id)
        scene.remove_world_object(box_id)
        scene.remove_world_object(cylinder_object.id)
        rospy.sleep(1)

        print " "
        print "============ Press `Enter` to move to Home and exit the program ..."
        raw_input()
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(1)
        joint_positions = [0.0, 0.0]
        gripper.set_joint_value_target(joint_positions)
        gripper.go()
        rospy.sleep(1)

        # 干净地关闭moveit_commander并退出程序
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    # 判断是否已成功添加物体到规划场景，或是否成功把物体附着到机械臂上
    def wait_for_state_update(self,obstacle_name,scene,obstacle_is_known=False, object_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        # 在4s的时间内，循环判断我们想要的规划场景状态是否更新成功，若成功，返回True
        while (seconds - start < timeout) and not rospy.is_shutdown():
          # 判断物体是否attach成功
          attached_objects = scene.get_attached_objects([obstacle_name])
          is_attached = len(attached_objects.keys()) > 0
          # 判断传入的物体是否已经在规划场景中，即物体是否添加成功
          is_known = obstacle_name in scene.get_known_object_names()
          # 判断是否是我们想要的规划场景更新状态
          if (object_is_attached == is_attached) and (obstacle_is_known == is_known):
            return True
          # sleep0.1s，为物体添加或附着留出时间。
          rospy.sleep(0.1)
          seconds = rospy.get_time()
        return False

if __name__ == "__main__":
    try:
        MoveItObstaclesDemo()
    except rospy.ROSInterruptException:
        pass
