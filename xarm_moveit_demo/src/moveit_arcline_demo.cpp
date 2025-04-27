#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

/*
  1. 将机械臂移动到Home位置
  2. 移动到圆弧起始点(0.4, 0.0, 0.45)
  3. 在y-z平面生成圆形路径：
  - 半径0.1米
  - 以0.015弧度步长离散化圆周
  4. 规划并执行笛卡尔路径
  5. 最后返回Home位置
 */

int main(int argc, char **argv)
{
    // 初始化ROS2节点
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "moveit_arcline_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // 创建异步执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]()
                               { executor.spin(); });

    // 创建MoveGroup接口
    moveit::planning_interface::MoveGroupInterface arm(node, "xarm");

    // 设置规划参数
    arm.allowReplanning(true);
    arm.setMaxVelocityScalingFactor(0.5);

    // 移动到Home位置
    RCLCPP_INFO(node->get_logger(), "Moving to pose: Home");
    arm.setNamedTarget("Home");
    arm.move();

    // 设置圆弧起始点
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp = node->now();
    target_pose.pose.position.x = 0.4;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.45;
    target_pose.pose.orientation.w = 1.0;

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);
    arm.move();

    // 在y-z平面内生成圆弧路径点
    std::vector<geometry_msgs::msg::Pose> waypoints;
    double centerA = target_pose.pose.position.y;
    double centerB = target_pose.pose.position.z;
    double radius = 0.1;

    // 以0.015弧度步长生成完整圆周
    for (double th = 0; th <= (3.1415926 * 2); th += 0.015)
    {
        target_pose.pose.position.y = centerA + radius * cos(th);
        target_pose.pose.position.z = centerB + radius * sin(th);
        waypoints.push_back(target_pose.pose);
    }

    // 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

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