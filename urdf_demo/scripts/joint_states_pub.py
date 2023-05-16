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
    rate = rospy.Rate(20)
    # 改变arm_1_joint、arm_4_joint、gripper_1_joint和gripper_2_joint的值，让机械臂摆动，手爪开合
    while not rospy.is_shutdown():
        for num in range(0,100):
            joint_state.header.stamp = rospy.Time.now()
            joint_state.position[0] = joint_state.position[0] + 0.02
            joint_state.position[3] = joint_state.position[3] - 0.015
            joint_state.position[6] = joint_state.position[6] + 0.0065
            joint_state.position[7] = joint_state.position[7] + 0.0065
            joint_state_pub.publish(joint_state)
            rate.sleep()
        for num in range(0,100):
            joint_state.header.stamp = rospy.Time.now()
            joint_state.position[0] = joint_state.position[0] - 0.02
            joint_state.position[3] = joint_state.position[3] + 0.015
            joint_state.position[6] = joint_state.position[6] - 0.0065
            joint_state.position[7] = joint_state.position[7] - 0.0065
            joint_state_pub.publish(joint_state)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joint_states_pub')
    try:
        rospy.loginfo('joint_states_pub node ...')
        demo()
    except rospy.ROSInterruptException:
        rospy.loginfo('joint_states_pub node initialize failed, please retry...')
