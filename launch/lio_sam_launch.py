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


    default_paramfile_path = os.path.join(
        plefix_this_pkg,
        'param', 'lio_sam.yaml'
    )


    rviz_path = os.path.join(
        plefix_this_pkg,
        'config', 'lio_sam.rviz'
    )


    config_params_file_path = LaunchConfiguration('params_file_path')


    declare_params_file_path = DeclareLaunchArgument(
        'params_file_path', default_value=default_paramfile_path,
        description='LIO_SAM 用のパラメータ YAML ファイルを指定します。'
    )

    ld.add_action(declare_params_file_path)


    node_imu_preintegration = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=[config_params_file_path],
        output='screen'
    )
    node_image_projection = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[config_params_file_path],
        output='screen'
    )
    node_feature_extraction = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[config_params_file_path],
        output='screen'
    )
    node_map_optimization = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[config_params_file_path],
        output='screen'
    )
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
    )

    ld.add_action(node_imu_preintegration)
    ld.add_action(node_image_projection)
    ld.add_action(node_feature_extraction)
    ld.add_action(node_map_optimization)
    ld.add_action(node_rviz2)


    return ld
