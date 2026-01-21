from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Lance ton monde Gazebo
    oakd_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 
                        'launch', 'start_oakd_world.launch.py')
        )
    )
    
    # Lance SLAM (Cartographer)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_cartographer'), 
                        'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )
    
    # Lance ton nœud de détection
    detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('inf_sim'), 
                        'launch', 'inference.launch.py')
        )
    )
    
    return LaunchDescription([
        oakd_world,
        slam,
        detection
    ])
