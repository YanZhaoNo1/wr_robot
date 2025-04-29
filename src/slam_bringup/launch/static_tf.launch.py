#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1','odom', 'base_link'],
            output='screen'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base',
            arguments=['0', '0', '0', '0', '0', '0','1','base_link', 'imu'],
            output='screen'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base',
            arguments=['-0.3954', '0', '0', '0', '0', '0', '1','base_link', 'laser'],
            output='screen'),
    ])

