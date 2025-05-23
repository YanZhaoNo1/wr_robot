
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os

def generate_launch_description():
    ## ***** Launch arguments *****
    load_state_filename_arg = DeclareLaunchArgument('load_state_filename',default_value='/home/zy/ws/wr_robot/src/ucar_navigation/maps/4.30.pbstream')

  ## ***** File paths ******
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'backpack_2d.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    ## ***** Nodes *****
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}],
        output = 'screen'
        )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': True}],
        arguments = [
            '-configuration_directory', FindPackageShare('ucar_navigation').find('ucar_navigation') + '/params/carto',
            '-configuration_basename', 'pure_localization.lua',
            '-load_state_filename', LaunchConfiguration('load_state_filename')],
        remappings = [
            ('scan', '/scan'),
            ('imu', '/imu'),
            ('odom', '/odom'),],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        )

    return LaunchDescription([
        # Launch arguments
        load_state_filename_arg,
        # Nodes
        robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
