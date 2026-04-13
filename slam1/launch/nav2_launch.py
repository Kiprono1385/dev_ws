from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/brian/dev_ws/src/slam1/maps/map.yaml'  # <-- your map.yaml
            }]
        ),

        # AMCL for localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=['/home/brian/dev_ws/src/slam1/config/nav2_params.yaml']
        ),

        # Lifecycle manager for localization
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),

        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=['/home/brian/dev_ws/src/slam1/config/nav2_params.yaml']
        ),

        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=['/home/brian/dev_ws/src/slam1/config/nav2_params.yaml']
        ),

        # Behavior tree / recovery behaviors are now integrated via BT Navigator
        # So we remove the old 'recoveries_server'

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=['/home/brian/dev_ws/src/slam1/config/nav2_params.yaml']
        ),

        # Lifecycle manager for navigation
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator'  # no recoveries_server needed
                ]
            }]
        ),
    ])
