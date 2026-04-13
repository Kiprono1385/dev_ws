from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='potato_disease_detection',
            executable='inference_engine',
            output='screen',
        ),
        Node(
            package='potato_disease_detection',
            executable='bridge',
            output='screen'
        ),
        Node(
            package='potato_disease_detection',
            executable='camera_viewer',
            output='screen'
        )
    ])
