"""
Simulator + drive mux.

Starts the normal simulator launch file, but with the simulator listening on
`ackermann_cmd` instead of `drive`, and puts the drive mux in between:

    drive, safety --> drive_mux --> ackermann_cmd --> simulator

All arguments of the simulator launch file still work, for example
    ros2 launch drive_mux sim_mux_launch.py map_path:=maps/levine_obs
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'launch', 'gym_bridge_launch.py')

    # the simulator, with its `drive` input renamed to `ackermann_cmd`
    simulator = GroupAction([
        SetRemap(src='/drive', dst='/ackermann_cmd'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch)),
    ])

    mux = Node(
        package='drive_mux',
        executable='drive_mux',
        name='drive_mux',
        parameters=[{'safety_timeout': LaunchConfiguration('safety_timeout')}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'safety_timeout',
            default_value='0.5',
            description='[s] how long drive stays blocked after the last '
                        'safety command.'),
        simulator,
        mux,
    ])
