from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch 参数
    joy_vel = LaunchConfiguration('joy_vel')
    joy_config_filename = LaunchConfiguration('joy_config_filename')
    joy_dev = LaunchConfiguration('joy_dev')
    publish_stamped_twist = LaunchConfiguration('publish_stamped_twist')

    # 拼接参数文件路径
    config_filepath = PathJoinSubstitution([
        get_package_share_directory('teleop_acker_joy'),
        'config',
        joy_config_filename
    ])

    return LaunchDescription([
        DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
        DeclareLaunchArgument('joy_config_filename', default_value='xbox.yaml'),
        DeclareLaunchArgument('joy_dev', default_value='0'),
        DeclareLaunchArgument('publish_stamped_twist', default_value='false'),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath, {'publish_stamped_twist': publish_stamped_twist}],
            remappings=[('/cmd_vel', joy_vel)],
        ),
    ])
