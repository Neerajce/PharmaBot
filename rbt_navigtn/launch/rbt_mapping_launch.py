import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': False}],
        arguments = [
    '-configuration_directory', FindPackageShare('rbt_navigtn').find('rbt_navigtn') + '/config',
    '-configuration_basename', 'cartographer.lua'],
        # remappings = [('/odom','/scan_odom')],
    output='screen'
    )

    cartographer_ocupncy_grd_nd = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': False},
            {'resolution': 0.05}],
        # remappings = [('/odom','/scan_odom')],
    )

    cartographer_config_dir = os.path.join(get_package_share_directory('rbt_navigtn'), 'config')
    cartographer_config_basename = 'cartographer.lua'

    config_rbt_navigtn_params = os.path.join(
        get_package_share_directory('rbt_navigtn'),
        'config',
        'rbt_navigtn_params.yaml',
    )
    # cartogrphr_params = 

    return LaunchDescription([
        cartographer_node,
        cartographer_ocupncy_grd_nd
    ])
