#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()


    plefix_this_pkg = get_package_share_directory('livox_bringup')


    default_configfile_path = os.path.join(
        plefix_this_pkg,
        'config', 'MID-360.json'
    )


    rviz_path = os.path.join(
        plefix_this_pkg,
        'config', 'MID-360.rviz'
    )


    config_format_type = LaunchConfiguration('format_type')
    config_use_rviz = LaunchConfiguration('use_rviz')
    config_pub_late = LaunchConfiguration('pub_late')
    config_config_json_path = LaunchConfiguration('config_json_path')


    parameters = [
        {"xfer_format": config_format_type},
        {"multi_topic": 0},
        {"data_src": 0},
        {"publish_freq": config_pub_late},
        {"output_data_type": 0},
        {"frame_id": 'livox_frame'},
        {"lvx_file_path": '~/livox_test.lvx'},
        {"user_config_path": config_config_json_path},
        {"cmdline_input_bd_code": 'livox0000000001'}
    ]


    declare_format_type = DeclareLaunchArgument(
        'format_type', default_value='0',
        description='パブリッシュされる Lidar のメッセージタイプを指定します。０：PointCloud2、１：カスタムポイントクラウト'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='起動時に Rviz2 によるプレビューの有無を指定します。True：Rviz2 が起動する、False：Rviz2 は起動しない。'
    )
    declare_pub_late = DeclareLaunchArgument(
        'pub_late', default_value='10.0',
        description='Lidar トピックの動作周波数を指定します。指定可能値：[5.0, 10.0, 20.0, 50.0, etc]'
    )
    declare_config_json_path = DeclareLaunchArgument(
        'config_json_path', default_value=default_configfile_path,
        description='使用する MID-360 のコンフィグ JSON ファイルパスを絶対パスで指定します。'
    )

    ld.add_action(declare_format_type)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_pub_late)
    ld.add_action(declare_config_json_path)


    node_livox = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=parameters
    )
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        condition=IfCondition(config_use_rviz)
    )

    ld.add_action(node_livox)
    ld.add_action(node_rviz2)

    
    return ld
