import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_desc = get_package_share_directory('turtlebot3_description')
    
    # Noeud Cartographer
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_desc, 'config'),
            '-configuration_basename', 'turtlebot3_cartographer.lua'
        ],
        remappings=[('/scan', '/scan'), ('/odom', '/odom')]
    )

    # Noeud Grid (Convertit la carte en pixels)
    grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'resolution': 0.05},
            {'publish_period_sec': 1.0}
        ]
    )

    return LaunchDescription([cartographer_node, grid_node])
