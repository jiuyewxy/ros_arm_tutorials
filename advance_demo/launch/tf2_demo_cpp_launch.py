from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('advance_demo'),
                    'launch',
                    'pub_marker_launch.py'
                ])
            ])
        ),
     Node(
            package="advance_demo",
            executable="tf_pub",
            remappings=[
                ('/tf_pub/marker_pose', '/marker_pose')],
            output='screen'
        ),
    Node(
            package="advance_demo",
            executable="tf_listen",
            output='screen'
        )

    ])

