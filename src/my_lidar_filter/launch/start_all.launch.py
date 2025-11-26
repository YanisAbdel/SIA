import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. On récupère le chemin vers le package de simulation officiel
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    # 2. On définit le lancement de la simulation (Gazebo)
    # On inclut le fichier "turtlebot3_world.launch.py" qui existe déjà
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 3. On définit le lancement du nœud (le filtre Lidar)
    lidar_filter_node = Node(
        package='my_lidar_filter',
        executable='filter',
        name='closest_point_filter',
        output='screen'
    )

    # 4. On définit le lancement de RViz2 pour voir les résultats
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 5. On retourne la description complète : Gazebo + Ton Filtre + RViz
    return LaunchDescription([
        gazebo_sim,
        lidar_filter_node,
        rviz_node
    ])
