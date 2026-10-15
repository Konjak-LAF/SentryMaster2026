import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _include_launch(package, launch_file, arguments=None):
    """按包名 + launch 文件名 include 子 launch，并透传参数。"""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
        ]),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    # 各节点参数文件默认取本包 config/ 下的 yaml，可通过同名 launch 参数覆盖
    declarations = [
        DeclareLaunchArgument(
            'uart_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('uart_driver'), 'config', 'driver_config.yaml']),
            description='uart_driver 参数文件',
        ),
        DeclareLaunchArgument(
            'planner_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('sentry_planner'), 'config', 'global_planning.yaml']),
            description='sentry_planner 参数文件',
        ),
        DeclareLaunchArgument(
            'decision_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('sentry_decision_sys'), 'config', 'decision.yaml']),
            description='sentry_decision_sys 参数文件',
        ),
        DeclareLaunchArgument(
            'odin_config_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('odin_ros_driver'), 'config', 'control_command.yaml']),
            description='odin_ros_driver 参数文件',
        ),
    ]

    # 机器人固定坐标变换发布节点（odin1_base_link -> vehicle）
    robot_description = Command(
        [
            "xacro ",
            os.path.join(
                get_package_share_directory("sentry_launch"),
                "urdf",
                "sentry_chassis.xacro",
            ),
        ]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "publish_frequency": 500.0}
        ],
    )

    return LaunchDescription(declarations + [
        # 机器人固定坐标变换发布节点
        robot_state_publisher,
        # Odin1 空间感知模块（里程计 / 点云 / tf）
        _include_launch('odin_ros_driver', 'odin1_ros2.launch.py', {
            'config_file': LaunchConfiguration('odin_config_file'),
        }),
        # USB 串口驱动（裁判系统/底盘数据 <-> 底盘控制）
        _include_launch('uart_driver', 'driver.launch.py', {
            'params_file': LaunchConfiguration('uart_params_file'),
        }),
        # 导航节点
        _include_launch('sentry_planner', 'global_planning.launch.py', {
            'params_file': LaunchConfiguration('planner_params_file'),
        }),
        # 决策节点
        _include_launch('sentry_decision_sys', 'launch.py', {
            'params_file': LaunchConfiguration('decision_params_file'),
        }),
    ])
