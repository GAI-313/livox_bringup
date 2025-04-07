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
                {'occupancy_max_z': 1.5},
                {'occupancy_min_z': -100.0},
                {'min_x_size': 100.0},
                {'min_y_size': 100.0},
                {'frame_id': 'camera_init'}
            ],
            remappings=[
                ('cloud_in', '/Laser_map'),
                ('octomap', '/octomap')
            ]
        )
    ])
