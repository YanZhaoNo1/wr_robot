from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions


def generate_launch_description():
    
    return LaunchDescription([
        launch_ros.actions.Node(
            package='wheeltec_n100_imu', 
            executable='imu_node', 
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'serial_baud': 921600,
                'debug': False,
                'imu_frame': 'imu',
                'yaw_offset': -2.094
            }],
            output='screen', ),
    ])
