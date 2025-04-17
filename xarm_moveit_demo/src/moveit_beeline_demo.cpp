#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>

/*
主要功能：
    1. 将机械臂移动到home位置
    2. 移动到三角形起点(0.4, 0.0, 0.5)
    3. 规划并执行三角形路径：
       - 向下移动20cm
       - 向右移动20cm
       - 返回起始点
    4. 最后返回home位置
 */

int main(int argc, char **argv)
{
    // 初始化ROS2节点
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "moveit_beeline_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // 创建异步执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]()
                               { executor.spin(); });

    // 创建MoveGroup接口
    moveit::planning_interface::MoveGroupInterface arm(node, "xarm");

    // 设置规划参数
    arm.allowReplanning(true);            // 规划失败重新规划
    arm.setMaxVelocityScalingFactor(0.8); // 速度调节到最大速度的0.8

    // 移动到Home位置
    RCLCPP_INFO(node->get_logger(), "Moving to pose: Home");
    arm.setNamedTarget("Home");
    arm.move();

    // 设置三角形第一个顶点位姿
    RCLCPP_INFO(node->get_logger(), "Moving to pose: target_pose");
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp = node->now();
    target_pose.pose.position.x = 0.4;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.5;
    target_pose.pose.orientation.w = 1.0;

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);
    arm.move();

    // 获取并保存当前位置
    geometry_msgs::msg::Pose start_pose = arm.getCurrentPose().pose;
    geometry_msgs::msg::Pose end_pose = start_pose;

    // 定义三角形路径点
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);

    geometry_msgs::msg::Pose wppose = start_pose;
    wppose.position.z -= 0.2; // 向下移动20cm
    waypoints.push_back(wppose);

    wppose.position.y += 0.2; // 向右移动20cm
    waypoints.push_back(wppose);

    waypoints.push_back(end_pose); // 返回起点

    // 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory; // 用于存储计算出的路径结果
    const double jump_threshold = 0.0;            // 跳跃阈值，用于检测路径中关节角度的突变
    const double eef_step = 0.01;                 // 末端执行器步长，表示路径中相邻路径点之间的最大距离，较小的点会生成更密集的路径点

    // 返回值成功规划的路径比例，1.0表示完全可以到达，0.8表示80%的路径可达剩余20%可能因碰撞或关节限制无法到达
    double fraction = arm.computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(node->get_logger(),
                "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    // 执行路径
    if (fraction == 1.0)
    {
        arm.execute(trajectory);
    }

    // 返回Home位置
    RCLCPP_INFO(node->get_logger(), "Moving to pose: Home");
    arm.setNamedTarget("Home");
    arm.move();

    // 关闭节点
    rclcpp::shutdown();
    spinner.join();
    return 0;
}