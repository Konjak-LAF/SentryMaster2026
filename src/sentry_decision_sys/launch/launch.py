"""Launch the sentry decision node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for the decision node."""

    decision_share = FindPackageShare("sentry_decision_sys")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [decision_share, "config", "decision.yaml"]
                ),
                description="Path to the decision parameter file.",
            ),
            Node(
                package="sentry_decision_sys",
                executable="sentry_decision_node",
                name="sentry_decision_node",
                parameters=[LaunchConfiguration("params_file")],
                output="both",
            ),
        ]
    )
