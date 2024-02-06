from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_path = FindPackageShare('advance_demo')
    default_rviz_config_path = PathJoinSubstitution([package_path, 'rviz', 'view.rviz'])
    return LaunchDescription([
        Node(
            package='advance_demo',
            executable='pub_marker',
            name='pub_marker'
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", default_rviz_config_path]
        ),
        Node(
            package="rqt_reconfigure",
            executable="rqt_reconfigure",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "/world","--child-frame-id", "/a_fixed_link", "--x", "-0.2",  "--y", "-0.2","--z", "0.1", "--yaw", "3.14"]
        )    
 ])