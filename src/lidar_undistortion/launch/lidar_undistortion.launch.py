import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    lidar_undistortion_node = launch_ros.actions.Node(
        package='lidar_undistortion', executable='lidar_undistortion_node', output='screen',
        parameters=[{
            'lidar_topic':'/scan_raw',
            'lidar_msg_delay_time': 10.0,
            'scan_direction_clockwise': True,
            'imu_topic': '/imu',
            'imu_frequency': 100.0,
            'output_frame_id': 'laser',
            'pub_raw_scan_pointcloud': True,
            'pub_laserscan': True,
            'laserscan_angle_increment': 0.004,
            'use_range_filter': True,
            'range_filter_min': 0.2,
            'range_filter_max': 12.0,
            'use_angle_filter': True,
            'angle_filter_min': -2.3,
            'angle_filter_max': 2.3,
            'use_radius_outlier_filter': True,
            'radius_outlier_filter_search_radius': 0.1,
            'radius_outlier_filter_min_neighbors': 2,
        }],
        remappings=[('/lidar_undistortion/scan','/scan')]
    )

    return LaunchDescription([
        lidar_undistortion_node,
    ])
