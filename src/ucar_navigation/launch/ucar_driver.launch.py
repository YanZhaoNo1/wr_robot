import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    rplidar_launch_file = os.path.join(
        FindPackageShare("rplidar_ros").find("rplidar_ros"), "launch", "rplidar_launch.py"
    )
    imu_launch_file = os.path.join(
        FindPackageShare("wheeltec_n100_imu").find("wheeltec_n100_imu"), "launch", "imu.launch.py"
    )
    wr_bot_launch_file = os.path.join(
        FindPackageShare("turn_on_dlrobot_robot").find("turn_on_dlrobot_robot"), "launch", "tank.launch.py"
    )
    static_tf_launch_file = os.path.join(
        FindPackageShare("ucar_bringup").find("ucar_bringup"), "launch", "static_tf_pub.launch.launch.py"
    )

    return LaunchDescription([
        # 启动 RPLidar
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file)
        ),

        # 启动 IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch_file)
        ),

        # 启动 机器人底盘
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(wr_bot_launch_file)
        ),

        # 启动静态 TF 变换
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_tf_launch_file)
        ),
    ])
