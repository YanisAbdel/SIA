import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Fix Graphique (Anti-Crash Ogre)
    fix_gfx = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')

    # 2. Chemins des dossiers
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # CHEMIN VERS LE URDF
   urdf_file_path = os.path.expanduser('~/src/Turtlebot3/projects/turtlebot3_descriptions/urdf/turtlebot3_burger_oak_d_pro.urdf')

    # 3. Lecture du fichier URDF
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # 4. Chemin vers ton monde
    world_file = os.path.expanduser('~/src/Turtlebot3/turtlebot3_gazebo/worlds/my_room.world')

    # --- GESTION DES MODÈLES ---
    downloaded_models_path = os.path.expanduser('~/.ignition/fuel/fuel.gazebosim.org/openrobotics/models')
    local_models_path = os.path.expanduser('~/src/Turtlebot3/turtlebot3_gazebo/models')
    all_models_paths = f"{downloaded_models_path}:{local_models_path}"
    set_res_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=all_models_paths)
    # ---------------------------

    # 5. Lancer Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 6. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        arguments=[urdf_file_path]
    )

    # 7. SPAWN DU ROBOT
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_burger_oak_d_pro',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen',
    )

    # 8. BRIDGE (PONT ROS2 <-> GAZEBO)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera/image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.TFMessage',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/rgb_camera/image', '/camera/image_raw'),
            ('/rgb_camera/camera_info', '/camera/camera_info'),
            ('/depth_camera/image', '/camera/depth/image_raw'),
            ('/depth_camera/camera_info', '/camera/depth/camera_info'),
            ('/depth_camera/image/points', '/camera/depth/points'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


    # Remplace la commande manuelle du Terminal 2
   # camera_tf = Node(
       # package='tf2_ros',
      #  executable='static_transform_publisher',
     #   arguments = ['0', '0', '0.', '0', '0', '0', 'oak_d_pro_depth_optical_frame', 'turtlebot3_burger_oak_d_pro/base_footprint/depth_camera'],
    #    parameters=[{'use_sim_time': True}],
    #    output='screen'
    #)
    # -----------------------------------------

    # 10. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('turtlebot3_descriptions'), 'rviz', 'model.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        fix_gfx,
        set_res_path, 
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
       # camera_tf,  
        rviz
    ])