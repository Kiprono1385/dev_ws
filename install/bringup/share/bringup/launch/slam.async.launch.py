from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                '/home/brian/dev_ws/src/bringup/config/mapper_params_online_async.yaml',
                {'use_sim_time': False}
            ]
        ),
    ])
