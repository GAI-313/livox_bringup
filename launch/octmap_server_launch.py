from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[
                {'resolution': 0.1},
                {'min_x_size': 1000.0},
                {'min_y_size': 1000.0},
                {'frame_id': 'map'}
            ],
            remappings=[
                ('cloud_in', '/glim_ros/map'),
                ('octomap', '/octomap')
            ]
        )
    ])