"""
Follow the gap on the map `levine_blocked` (empty loop, timed laps).

Starts only our own node. The simulator must already be running:
    ros2 launch f1tenth_gym_ros gym_bridge_launch.py map_path:=maps/levine_blocked

Run
    ros2 launch gap_follow levine_blocked_launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

EXECUTABLE = 'reactive_node'

# No obstacles on this map, so the tuning can be fast.
PARAMETERS = {
    'speed': 3.0,            # [m/s] top speed
    'max_range': 8.0,        # [m]   further than this counts as equally far
    'bubble_radius': 0.3,    # [m]   safety bubble around the closest point
    'clearance': 0.18,       # [m]   half car width + margin
    'far_ratio': 0.95,       # [-]   "far" = at least this share of the deepest beam
}


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gap_follow',
            executable=EXECUTABLE,
            name='reactive_node',
            parameters=[PARAMETERS],
            output='screen',
        ),
    ])
