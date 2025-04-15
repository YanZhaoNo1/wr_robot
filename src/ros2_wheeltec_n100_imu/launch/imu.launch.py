from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions


def generate_launch_description():
    
    return LaunchDescription([
        launch_ros.actions.Node(
            package='wheeltec_n100_imu', 
            executable='imu_node', 
            output='screen', ),
    ])
