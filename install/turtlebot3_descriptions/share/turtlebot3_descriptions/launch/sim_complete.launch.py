import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. CONFIGURATION (Fix Graphique VM)
    fix_gfx = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')

    # Chemins
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_descriptions = get_package_share_directory('turtlebot3_descriptions')
    
    # Monde et Modèles
    world_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )
    models_path = os.path.expanduser('~/turtlebot3_ws/src/Turtlebot3/turtlebot3_gazebo/models')
    set_models_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path)

    # 2. SIMULATEUR + ROBOT
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    robot_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_descriptions, 'launch', 'turtlebot3_oak_d_pro.launch.py')
        )
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'oakd_bot', 
                   '-x', '-2.0', '-y', '-0.5', '-z', '0.05'],
        output='screen'
    )

    # --- PONT (BRIDGE) MIS À JOUR ---
    # On connecte : 
    # 1. Le Lidar (/scan) : Gazebo -> ROS
    # 2. Les Moteurs (/cmd_vel) : ROS -> Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    # =======================================================================
    # 3. OBJETS DU MONDE
    # =======================================================================

    # POUBELLES (Dans les coins intérieurs)
    poubelle_1 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'poubelle_A', '-x', '1.5', '-y', '1.5', '-z', '0.0',
                   '-file', 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/TrashBin'],
        output='screen'
    )
    poubelle_2 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'poubelle_B', '-x', '1.5', '-y', '-1.5', '-z', '0.0',
                   '-file', 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/TrashBin'],
        output='screen'
    )
    poubelle_3 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'poubelle_C', '-x', '-1.5', '-y', '1.5', '-z', '0.0',
                   '-file', 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/TrashBin'],
        output='screen'
    )

    # GAMELLES (Design Gris/Bleu)
    gamelle_design = """
    <sdf version='1.7'>
      <model name='gamelle_realiste'>
        <static>true</static>
        <link name='bol'>
           <visual name='visu_metal'>
             <geometry><cylinder><radius>0.12</radius><length>0.04</length></cylinder></geometry>
             <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
           </visual>
           <collision name='col_metal'>
             <geometry><cylinder><radius>0.12</radius><length>0.04</length></cylinder></geometry>
           </collision>
           <visual name='visu_eau'>
             <pose>0 0 0.002 0 0 0</pose>
             <geometry><cylinder><radius>0.10</radius><length>0.041</length></cylinder></geometry>
             <material><ambient>0 0 0.8 1</ambient><diffuse>0 0 0.8 1</diffuse></material>
           </visual>
        </link>
      </model>
    </sdf>
    """

    gamelle_1 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'gamelle_A', '-x', '0.5', '-y', '0.0', '-z', '0.0', '-string', gamelle_design],
        output='screen'
    )
    gamelle_2 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'gamelle_B', '-x', '-1.0', '-y', '1.5', '-z', '0.0', '-string', gamelle_design],
        output='screen'
    )
    gamelle_3 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'gamelle_C', '-x', '-1.0', '-y', '-1.5', '-z', '0.0', '-string', gamelle_design],
        output='screen'
    )

    return LaunchDescription([
        fix_gfx, set_models_path,
        gazebo, robot_desc, spawn_robot, bridge,
        poubelle_1, poubelle_2, poubelle_3,
        gamelle_1, gamelle_2, gamelle_3
    ])
