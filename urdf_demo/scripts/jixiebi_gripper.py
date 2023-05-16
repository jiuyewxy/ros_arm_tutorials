#!/usr/bin/env python
#coding=utf-8

import rospy
from sensor_msgs.msg import JointState

def demo():
    # 定义发布/joint_states话题
    joint_state_pub = rospy.Publisher('/joint_states', JointState , queue_size=10)
    # 定义关节名称和初始位置
    joint_state = JointState()
    joint_state.name = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint" , "arm_5_joint","arm_6_joint", "gripper_1_joint", "gripper_2_joint"]
    joint_state.position = [0, 0, 0, 0, 0, 0, 0, 0]
    # 定义用来标志正向旋转的标志
    forward = True

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        joint_state.header.stamp = rospy.Time.now()
        # 改变arm_joint_2的值，让机械臂摆动
        if forward == True and joint_state.position[1] <= 1.5:
            joint_state.position[1] = joint_state.position[1] + 0.015
             # 定义用来标志手爪正向旋转的标志
            forward2 = True
        elif forward == True and joint_state.position[1] > 1.5:
             # 改变arm_joint_5的值，让手爪开合
            if joint_state.position[6] <= 0.9 and\
            forward2 == True:
                joint_state.position[6] = joint_state.position[6] + 0.015
                joint_state.position[7] = joint_state.position[7] + 0.015
            elif joint_state.position[6] >= 0:
                forward2 = False
                joint_state.position[6] = joint_state.position[6] - 0.015
                joint_state.position[7] = joint_state.position[7] - 0.015
                if joint_state.position[6] < 0:
                    forward = False
        elif forward == False and joint_state.position[1] >= 0:
            joint_state.position[1] = joint_state.position[1] - 0.015
        else:
            forward = True
        joint_state_pub.publish(joint_state)
        rate.sleep()



if __name__ == '__main__':
    rospy.init_node('urdf_state_publisher_demo')
    try:
        rospy.loginfo('Using urdf with robot_state_publisher ...')
        demo()
    except rospy.ROSInterruptException:
        rospy.loginfo('urdf_state_publisher_demo node initialize failed, please retry...')
