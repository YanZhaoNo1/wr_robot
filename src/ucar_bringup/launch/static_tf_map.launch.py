#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom',
            arguments=['0', '0', '0', '0.924', '-0.383', '0', '0','map', 'odom'], # x 0.924 y -0383 
            output='screen'),
    ])

