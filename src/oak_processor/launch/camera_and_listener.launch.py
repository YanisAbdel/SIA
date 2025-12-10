import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Définir le chemin VERS LE FICHIER YAML (chemin absolu)
    # Note: os.path.expanduser('~') est utilisé pour résoudre le problème du '~'
    config_file_path = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'enable_pcl.yaml'
    )
    
    # 2. Chemin vers le package du driver
    driver_pkg_dir = get_package_share_directory('depthai_ros_driver')
    
    # --- Composants du Launch ---
    
    # A. Lancement du Driver OAK-D (Le PUBLISHER)
    # On utilise le fichier camera.launch.py et on lui passe votre fichier YAML
    oak_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_pkg_dir, 'launch', 'camera.launch.py')
        ),
        # On passe le fichier YAML en tant qu'argument de lancement
        launch_arguments={'params_file': config_file_path}.items(),
    )
    
    # B. Lancement du Listener Python (Votre SUBSCRIBER)
    custom_listener_node = Node(
        package='oak_processor',
        executable='oak_listener',
        name='oak_listener_node',
        output='screen'
    )
    
    # C. Message de confirmation
    log_message = LogInfo(
        msg="ROS2 Camera pipeline active. Check separate OpenCV windows for streams."
    )

    return LaunchDescription([
        oak_driver_launch,
        custom_listener_node,
        log_message,
    ])
