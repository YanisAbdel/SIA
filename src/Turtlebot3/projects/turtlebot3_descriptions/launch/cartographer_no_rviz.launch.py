import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # On force use_sim_time à true car on est sur Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # On récupère le chemin vers ton package
    pkg_share = get_package_share_directory('turtlebot3_descriptions')
    
    # Chemin vers le dossier config et le fichier Lua
    configuration_directory = os.path.join(pkg_share, 'config')
    configuration_basename = 'turtlebot3_cartographer.lua'

    # Le nœud principal de Cartographer (Le cerveau SLAM)
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename],
    )

    # Le nœud qui transforme les données brutes en une carte (Grille)
    # C'est INDISPENSABLE pour pouvoir sauvegarder la map ensuite
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'resolution': 0.05}], # Résolution de 5cm par pixel
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node
    ])
