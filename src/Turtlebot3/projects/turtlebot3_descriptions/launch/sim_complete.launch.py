import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    fix_gfx = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')


    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_descriptions = get_package_share_directory('turtlebot3_descriptions')
    

    world_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )
    models_path = os.path.expanduser('~/turtlebot3_ws/src/Turtlebot3/turtlebot3_gazebo/models')
    set_models_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path)


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


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Lidar
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            # Moteurs
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Caméra Couleur (RGB)
            '/rgb_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # Caméra de Profondeur (Depth)
            '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # Nuage de Points (Point Cloud 3D)
            '/depth_camera/image/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # Odométrie
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        remappings=[
            ('/rgb_camera/image', '/camera/image_raw'),
            ('/rgb_camera/camera_info', '/camera/camera_info'),
            ('/depth_camera/image', '/camera/depth/image_raw'),
            ('/depth_camera/camera_info', '/camera/depth/camera_info'),
            ('/depth_camera/image/points', '/camera/depth/points')
        ],
        output='screen'
    )


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
