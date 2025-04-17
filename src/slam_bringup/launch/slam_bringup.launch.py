import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取 rplidar_ros 和 wheeltec_n100_imu 的 launch 文件路径
    rplidar_launch_file = os.path.join(
        FindPackageShare("rplidar_ros").find("rplidar_ros"), "launch", "rplidar_launch.py"
    )
    imu_launch_file = os.path.join(
        FindPackageShare("wheeltec_n100_imu").find("wheeltec_n100_imu"), "launch", "imu.launch.py"
    )
    static_tf_launch_file = os.path.join(
        FindPackageShare("slam_bringup").find("slam_bringup"), "launch", "static_tf.launch.py"
    )
    lidar_undistortion_launch_file = os.path.join(
        FindPackageShare("lidar_undistortion").find("lidar_undistortion"), "launch", "lidar_undistortion.launch.py"
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

        # 启动雷达去畸变
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(lidar_undistortion_launch_file)
        # ),
        # 启动静态 TF 变换
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_tf_launch_file)
        ),


    ])
