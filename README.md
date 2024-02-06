# 《ROS机械臂开发与实践》教材源码

## 说明

本分支是《ROS机械臂开发与实践》教学代码包的ROS2 Humble版本，教材正文讲解以 ROS-Melodic 版本为准， 基于“如何从零开始搭建机械臂的 ROS 控制系统”这一问题，由浅入深，由易到难，理论结合实践，详细介绍了 ROS 机械臂开发过程中使用的技术，并通过大量原创工程实例，帮助读者深入理解 ROS 框架，学会将 ROS 和 MoveIt!应用到具体的机器人开发实践中。

ros_arm_tutorials 各功能包简要说明如下（目前已完成前五章代码的ROS2版本适配，后续会持续更新其他代码并在openEuler系统上进行测试）：

| 软件包              | 内容                                                         |
| ------------------- | ------------------------------------------------------------ |
| base_demo           | 自定义消息和服务、topic发布/订阅节点、service服务端/客户端节点、参数操作示例节点 |
| advance_demo        | action 的定义和服务端/客户端节点、ROS 常用工具、动态参数配置节点、TF2示例节点、RVIZ Marker发布和显示等 |
| myrobot_description | 三自由度机械臂和移动小车的URDF模型                           |
| darm                | Solidworks 导出的 XBot-Arm 机械臂原始 URDF 模型文件包        |
| xarm_description    | XBot-Arm 机械臂 URDF 模型文件包                              |
| urdf_demo           | URDF 模型和 robot_state_publisher 节点的使用示例             |
| xarm_driver         | XBot-Arm 真实机械臂驱动包                                    |
| xarm_moveit_config  | 使用 配置助手生成的 XBot-Arm 机械臂 MoveIt!配置和启动功能包  |
| xarm_moveit_demo    | 使用 MoveIt!的编程接口实现路径规划、避障以及机械臂的抓取和放置等示例 |
| xarm_vision         | 摄像头启动、相机标定、颜色检测、AR标签识别、手眼标定、自动抓取与放置示例 |

<img src="img/ROS机械臂开发与实践.png" style="zoom:60%;" />

## 部分教学示例展示

下面是ROS2 Humble版本部分示例效果图：

<img src="img/ros2.jpeg" style="zoom:67%;" />



## Copyright

<img src="img/logo.png" style="zoom:80%;" />