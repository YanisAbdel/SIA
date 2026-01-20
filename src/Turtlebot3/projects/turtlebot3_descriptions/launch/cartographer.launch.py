import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = get_package_share_directory('turtlebot3_descriptions')
    configuration_directory = os.path.join(pkg_share, 'config')
    configuration_basename = 'turtlebot3_cartographer.lua'

    cartographer_node = Node(
        package='cartographer_ros', executable='cartographer_node',
        name='cartographer_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory, '-configuration_basename', configuration_basename],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros', executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'resolution': 0.05}],
    )

    return LaunchDescription([cartographer_node, occupancy_grid_node])
