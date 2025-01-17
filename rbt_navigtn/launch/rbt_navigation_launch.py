import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('rbt_navigtn'), 'config')
    map_dir = os.path.join(
        get_package_share_directory('rbt_navigtn'),'map')
    
    map_file = os.path.join(map_dir,'xyz.yaml')
    param_file = os.path.join(config_dir,'rbt_navigtn_params.yaml')

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_to_odom',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']    
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'nav2_bringup'),'launch','bringup_launch.py')
            ),
            launch_arguments={
                'map':map_file,
                'params_file':param_file,
                'use_sim_time':'False'
            }.items(),
        )
    ])

if __name__=='__main__':
    generate_launch_description()