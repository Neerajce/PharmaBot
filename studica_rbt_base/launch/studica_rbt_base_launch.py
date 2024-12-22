import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('studica_rbt_base'),
        'config',
        'studica_rbt_base_params.yaml',
    )

    return LaunchDescription([
    ])
