#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rospy
import rosparam
def param_demo():
    # 初始化节点
    rospy.init_node('param_demo')
    # 设置参数
    rospy.set_param('/a_string', 'hello word')
    rospy.set_param('list_of_floats', [1., 2.1, 3.2, 4.3])
    rospy.set_param('~private_int', 2)
    rosparam.set_param('bool_True', "true")
    rosparam.set_param('gains', "{'p': 1, 'i': 2, 'd': 3}")
    # 获取当前所有参数的名字
    try:
        names = rospy.get_param_names()
        rospy.loginfo('Param names: %s',names)
    except ROSException:
        rospy.logerr('could not get param name')
    # 获取参数的值
    string_param = rospy.get_param('/a_string')
    rospy.loginfo('Param /a_string : %s', string_param)
    list_param = rospy.get_param('list_of_floats')
    rospy.loginfo('Param list_of_floats :[%.1f, %.1f, %.1f, %.1f]', list_param[0], list_param[1],list_param[2],list_param[3])
    gains = rosparam.get_param('gains')
    p, i, d = gains['p'], gains['i'], gains['d']
    rospy.loginfo('p: %i, i: %i, d: %i', p, i, d)
    # 从最近的命名空间查找参数并返回参数的完整名字
    param_name = rospy.search_param('private_int')
    int_param = rosparam.get_param(param_name)
    rospy.loginfo('Param %s : %i', param_name, int_param)

    # 获取参数的值,若参数服务器中没有该参数，则使用默认值
    default_param = rospy.get_param('default_param', 100)
    rospy.loginfo('default_param: %i',default_param)
    # 删除参数
    try:
        # rosparam.delete_param('/a_string')
        rospy.delete_param('/a_string')
    except KeyError:
        rospy.logerr('/a_string value not set')
    # 判断参数是否存在
    if rospy.has_param('/a_string'):
        rospy.logerr('Failed to delete param /a_string ')
    else:
        rospy.loginfo('Successfully deleted param /a_string ')

if __name__ == '__main__':
    param_demo()

