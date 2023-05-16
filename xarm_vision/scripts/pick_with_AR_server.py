#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
import math
from std_msgs.msg import Int8
from xarm_vision.srv import *
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp,PlaceLocation,GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from copy import deepcopy

GROUP_NAME_ARM = 'xarm'
GROUP_NAME_GRIPPER = 'gripper'
GRIPPER_FRAME = 'gripper_centor_link'
GRIPPER_OPEN = [0.68,0.68]
GRIPPER_GRASP = [0.25,0.25]
GRIPPER_CLOSED = [0.0,0.0]
GRIPPER_JOINT_NAMES = ['gripper_1_joint','gripper_2_joint']
GRIPPER_EFFORT = [1.0,1.0]
REFERENCE_FRAME = 'base_link'

class ARTrackAndPick:
    def __init__(self):
        # 初始化Python API 依赖的moveit_commanderC++系统
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化节点
        rospy.init_node('pick_with_AR_server')

        # 话题ar_pose_marker的订阅端，回调函数为get_tags
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.get_tags)

        # 初始化tag_result为AlvarMarkers消息类型
        self.tag_result = AlvarMarkers()

        # 初始化场景对象，用来在规划场景中添加或移除物体
        self.scene = PlanningSceneInterface()
        rospy.sleep(1)

        # 定义ROS服务/xarm_vision_pickup的服务端，服务回调函数为call_pick_place()
        self.pick_place_srv = rospy.Service('/xarm_vision_pickup', CallPickPlaceDemo, self.call_pick_place)

        # 初始化规划组
        self.xarm = MoveGroupCommander(GROUP_NAME_ARM)
        self.gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        # 机械臂先回到初始位置
        self.xarm.set_named_target('Home')
        self.xarm.go()
        rospy.sleep(1)

        self.table_id = "table"

        rospy.loginfo("Pick and Place demo is ready. You can call the service /xarm_vision_pickup to test... ")
        rospy.spin()

        # 干净地关闭moveit_commander并退出程序
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    # ar_pose_marker话题回调函数
    def get_tags(self, msg):
        self.tag_result = msg

    # /xarm_vision_pickup的服务回调函数
    def call_pick_place(self, req):
        n = len(self.tag_result.markers)
        # 如果没有检测到标签，服务响应的success为False
        if n == 0:
            print "No target found!!!"
            return CallPickPlaceDemoResponse(False)

        # 设置桌子的长宽高 [l, w, h]
        table_size = [1.0, 1.2, 0.01]
        # 设置桌子的位姿
        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = - table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        # 把桌子添加到规划场景中
        self.scene.add_box(self.table_id, table_pose, table_size)
        rospy.sleep(1)
        # 设置桌子table为抓取和放置操作的支撑面，使MoveIt!忽略物体放到桌子上时产生的碰撞警告
        self.xarm.set_support_surface_name(self.table_id)

        target_poses = [] # 保存标签位姿
        target_ids = [] # 保存标签的ID
        yaw_offset = [] # 保存偏航角偏移量，后面用于设置放置姿态
        target_size = [0.07,0.07, 0.07] # 方块尺寸

        # 在规划场景中添加所有的目标物体，并记录每个目标的位姿
        for tag in self.tag_result.markers:
            # target_ids列表用来保存每个标签的ID，作为规划场景中目标物体的ID
            tag.pose.header.frame_id = REFERENCE_FRAME
            target_ids.append(deepcopy(str(tag.id)))
            # target_poses列表用来保存每个标签的位姿pose
            target_poses.append(deepcopy(tag.pose))
            # 设置方块的尺寸和中心位置，这里以方块表面的AR标签的高度作为边长
            tag.pose.pose.position.z = tag.pose.pose.position.z/2.0
            # 获取AR标签的姿态，从四元数转为欧拉角表示
            rpy_euler = euler_from_quaternion([tag.pose.pose.orientation.x, tag.pose.pose.orientation.y,tag.pose.pose.orientation.z,tag.pose.pose.orientation.w ])
            # yaw_offset列表用于保存每个目标方块的偏航角与抓取时的gripper_centor_link的偏航角yaw的差值
            yaw = math.atan(tag.pose.pose.position.y/tag.pose.pose.position.x)
            yaw_offset.append(deepcopy(rpy_euler[2]-yaw))
            # 设置方块的姿态与欧拉角的Yaw偏航角一致，Roll和Pitch为零
            q = quaternion_from_euler(0, 0, rpy_euler[2])
            tag.pose.pose.orientation.x = q[0]
            tag.pose.pose.orientation.y = q[1]
            tag.pose.pose.orientation.z = q[2]
            tag.pose.pose.orientation.w = q[3]
            # 在规划场景中添加目标方块
            self.scene.add_box(str(tag.id), tag.pose, target_size)
        # 给规划场景一定的更新时间
        rospy.sleep(1)

        # 设置一个放置目标位姿place_pose
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_pose.pose.position.x = 0.25
        place_pose.pose.position.y = 0.25
        place_pose.pose.position.z = target_size[2] / 2.0

        # 抓取目标并放置到指定位置
        for i in range(len(target_poses)):
            q = quaternion_from_euler(0, 0, math.pi/4 + yaw_offset[i])
            place_pose.pose.orientation.x = q[0]
            place_pose.pose.orientation.y = q[1]
            place_pose.pose.orientation.z = q[2]
            place_pose.pose.orientation.w = q[3]
            if self.pick_and_place(target_ids[i], target_poses[i], place_pose):
                place_pose.pose.position.z += 0.07
                continue
            else:
                return CallPickPlaceDemoResponse(False)

        # 回到初始位姿
        self.xarm.set_named_target('Home')
        self.xarm.go()
        # 闭合手爪
        self.gripper.set_joint_value_target(GRIPPER_CLOSED)
        self.gripper.go()
        rospy.sleep(1)
        # 删除规划场景里的物体
        self.scene.remove_world_object(self.table_id)
        for i in range(len(target_ids)):
            self.scene.remove_world_object(target_ids[i])
        rospy.sleep(1)
        # 返回服务的响应success为True
        return CallPickPlaceDemoResponse(True)



    def pick_and_place(self,target_id,target_pose,place_pose):
        # 获取 end-effector link的名字
        end_effector_link = self.xarm.get_end_effector_link()

        # 允许重规划
        self.xarm.allow_replanning(True)

        # 设置目标的参考系
        self.xarm.set_pose_reference_frame(REFERENCE_FRAME)

        # 设置每次规划尝试的时间为5s
        self.xarm.set_planning_time(5)

        # 定义抓取操作的最大尝试次数
        max_pick_attempts = 5

        # 定义放置操作的最大尝试次数
        max_place_attempts = 5

        # 初始化抓取的目标位姿
        grasp_pose = target_pose
        grasp_pose.pose.position.z = 0.08

        # 考虑到抓取方块时，希望手爪竖直向下，能夹住方块的两边，
        # 所以需要设置gripper_centor_link的位置位于方块中心上方，姿态为(0, math.pi/2, 0)
        q = quaternion_from_euler(0, math.pi/2, 0)
        grasp_pose.pose.orientation.x = q[0]
        grasp_pose.pose.orientation.y = q[1]
        grasp_pose.pose.orientation.z = q[2]
        grasp_pose.pose.orientation.w = q[3]

        # 生成grasp抓取列表
        grasps = self.make_grasps(grasp_pose)

        # 设置result标记每次抓取尝试的结果
        result = None
        n_attempts = 0

        # 循环尝试抓取操作直到抓取成功或最大尝试次数用尽
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = self.xarm.pick(target_id, grasps)
            rospy.sleep(0.2)

        # 如果抓取成功，尝试物品放置操作。
        if result == MoveItErrorCodes.SUCCESS:
            # 重置标记操作结果的result和尝试次数
            result = None
            n_attempts = 0

            # 生成一系列放置位姿
            places = self.make_places(place_pose)

            # 循环放置直到放置成功或超过最大尝试次数
            while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
                n_attempts += 1
                rospy.loginfo("Place attempt: " +  str(n_attempts))
                for place in places:
                    result = self.xarm.place(target_id,place)
                    if result == MoveItErrorCodes.SUCCESS:
                        return True
                rospy.sleep(0.2)

            if result != MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
                return False
        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")
            return False

    # 通过手爪关节的位置生成JointTrajectory消息类型的轨迹返回值
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()

        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES

        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()

        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions

        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT

        tp.time_from_start = rospy.Duration(5)

        # Append the goal point to the trajectory points
        t.points.append(tp)

        # Return the joint trajectory
        return t

    # 通过传入的向量vector和最小距离、期望距离，生成GripperTranslation消息类型并返回
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()

        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        # The vector is relative to the gripper frame
        g.direction.header.frame_id = REFERENCE_FRAME

        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

    # 生成一系列可能的抓取列表
    def make_grasps(self, initial_pose_stamped):
        # 初始化抓取列表
        grasps = []
        # 把变量g初始化为Grasp消息类型的对象
        g = Grasp()

        # 设置 pre_grasp_posture和grasp_posture
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_GRASP)

        # 设置靠近和撤离的方向以及距离
        g.pre_grasp_approach = self.make_gripper_translation(0.06, 0.1, [0, 0.0,-1.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.06, 0.1, [0.0, 0.0, 1.0])

        # 设置grasp_pose
        g.grasp_pose = initial_pose_stamped

        # Pitch angles to try
        yaw_vals = [0,0.03,0.05,-0.03,-0.05]

        # A list to hold the grasps
        grasps = []

        # 设置抓取的初始姿态为机械臂末端比较容易到达的位姿
        yaw = math.atan(g.grasp_pose.pose.position.y/g.grasp_pose.pose.position.x)
        # Generate a grasp for each pitch and yaw angle
        for y in yaw_vals:
            # Create a quaternion from the Euler angles
            q = quaternion_from_euler(0, math.pi/2, yaw + y)

            # Set the grasp pose orientation accordingly
            g.grasp_pose.pose.orientation.x = q[0]
            g.grasp_pose.pose.orientation.y = q[1]
            g.grasp_pose.pose.orientation.z = q[2]
            g.grasp_pose.pose.orientation.w = q[3]

            # Set and id for this grasp (simply needs to be unique)
            g.id = str(len(grasps))

            # Set the allowed touch objects to the input list
            g.allowed_touch_objects = self.table_id

            # Don't restrict contact force
            g.max_contact_force = 0

            # Append the grasp to the list
            grasps.append(deepcopy(g))

        # 把此grasp添加到grasps列表
        grasps.append(deepcopy(g))

        # 返回
        return grasps

    # 生成一系列可能的放置点位姿
    def make_places(self, init_pose):
        # Initialize the place location as a PlaceLocation message
        place = PlaceLocation()
        # Start with the input place pose
        place.place_pose = init_pose
        # 设置靠近放置点的方向、最小移动距离和期望距离，这里设置为沿着Z轴向下移动0.08米
        place.pre_place_approach.direction.header.frame_id = REFERENCE_FRAME;
        place.pre_place_approach.direction.vector.z = -1.0;
        place.pre_place_approach.min_distance = 0.05;
        place.pre_place_approach.desired_distance = 0.08;
        # 设置放置完成后机械臂的撤离方向、移动最小距离和期望距离，这里设置为沿着Z轴向上移动0.08米
        place.post_place_retreat.direction.header.frame_id = REFERENCE_FRAME;
        place.post_place_retreat.direction.vector.z = 1.0;
        place.post_place_retreat.min_distance = 0.06;
        place.post_place_retreat.desired_distance = 0.08;

        # A list of x shifts (meters) to try
        x_vals = [0, 0.005, -0.005]

        # A list of y shifts (meters) to try
        y_vals = [0, 0.005, -0.005]

        # A list to hold the places
        places = []

        # Generate a place pose for each angle and translation
        for y in y_vals:
            for x in x_vals:
                place.place_pose.pose.position.x = init_pose.pose.position.x + x
                place.place_pose.pose.position.y = init_pose.pose.position.y + y
                # Append this place pose to the list
                places.append(deepcopy(place))

        # Return the list
        return places

if __name__ == "__main__":
    ARTrackAndPick()
