import os
import sys
from glob import glob
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    share_dir = get_package_share_directory('data_logger')
    idx = share_dir.find("/install/data_logger/share/data_logger")
    ws_dir = share_dir[:idx]
    list = [
        Node(
            package='data_logger',
            executable='data_logger',
            namespace='',
            # output="screen",
            # arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[{"log_dir" : os.path.join(ws_dir,"data_log")}],
            respawn=True,
        )
    ]

    return LaunchDescription(list)