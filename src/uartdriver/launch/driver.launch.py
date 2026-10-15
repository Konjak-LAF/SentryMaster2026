import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _create_driver_node(context):
    params_file = LaunchConfiguration("params_file").perform(context)

    return [
        Node(
            package='uart_driver',
            executable='uart_driver_node',
            name='uart_driver_node',
            parameters=[params_file],
            output='both',
        )
    ]


def generate_launch_description():
    driver_share = FindPackageShare("uart_driver")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([driver_share, "config", "driver_config.yaml"]),
            description="Path to the uart_driver parameter file.",
        ),
        OpaqueFunction(function=_create_driver_node),
    ])
