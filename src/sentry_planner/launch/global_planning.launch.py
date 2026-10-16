from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _create_planner_node(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    parameters = [
        params_file,
    ]

    # 真车实测标定结果（tools/calib_speed.py 生成）：存在时叠加为第二个参数文件，
    # 只覆盖速度/加速度上限，其余参数仍来自 global_planning.yaml（ROS2 后加载的生效）
    speed_limits_file = Path(LaunchConfiguration("speed_limits_file").perform(context))
    if speed_limits_file.is_file():
        parameters.append(str(speed_limits_file))

    return [
        Node(
            package="sentry_planner",
            executable="sentry_planner_node",
            name="sentry_planner_node",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    planner_share = FindPackageShare("sentry_planner")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [planner_share, "config", "global_planning.yaml"]
                ),
                description="Path to the planner parameter file.",
            ),
            DeclareLaunchArgument(
                "speed_limits_file",
                default_value=PathJoinSubstitution(
                    [planner_share, "config", "speed_limits.yaml"]
                ),
                description="真车功率/速度标定结果（存在则覆盖速度与加速度上限）。",
            ),
            OpaqueFunction(function=_create_planner_node),
        ]
    )
