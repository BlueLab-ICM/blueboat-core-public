"""Start the recording and monitoring nodes. Each restarts on its own if it exits."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Return one respawning node per job."""
    return LaunchDescription(
        [
            Node(
                package='core_ops',
                executable=executable,
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            )
            for executable in ('recorder_node', 'telemetry_node', 'monitor_node')
        ]
    )
