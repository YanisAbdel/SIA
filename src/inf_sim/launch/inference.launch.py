from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='inf_sim',
            executable='inference_node',
            name='inference_node',
            output='screen',
            parameters=[
                # ON BRANCHE SUR LES TUYAUX ACTIFS (Vus dans ton topic list)
                {'topic_rgb': '/camera/image_raw'},
                {'topic_depth': '/camera/depth/image_raw'},
                {'topic_info': '/camera/camera_info'},
                {'use_sim_time': True}
            ]
        )
    ])