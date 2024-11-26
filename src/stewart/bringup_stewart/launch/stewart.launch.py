from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    ld = LaunchDescription() # Begin building a launch description

    #################### Camera Driver Node #####################
    # camera_driver_pkg_prefix = get_package_share_directory('camera_driver')
    # camera_driver_param_file = os.path.join(
    #     camera_driver_pkg_prefix, 'config', 'params.yaml')
    
    # camera_driver_param = DeclareLaunchArgument(
    #     'camera_driver_param_file',
    #     default_value=camera_driver_param_file,
    #     description='Path to config file for perception node'
    # )
    # camera_driver_node = Node(
    #     package='camera_driver',
    #     name='camera_driver_node',
    #     executable='camera_driver_node',
    #     parameters=[LaunchConfiguration('camera_driver_param_file')],
    # )
    # ld.add_action(camera_driver_param)
    # ld.add_action(camera_driver_node)

    # #################### Perception Node #####################
    # perception_pkg_prefix = get_package_share_directory('perception')
    # perception_param_file = os.path.join(
    #     perception_pkg_prefix, 'config', 'params.yaml')
    
    # perception_param = DeclareLaunchArgument(
    #     'perception_param_file',
    #     default_value=perception_param_file,
    #     description='Path to config file for perception node'
    # )
    # perception_node = Node(
    #     package='perception',
    #     name='perception_node',
    #     executable='perception_node',
    #     parameters=[LaunchConfiguration('perception_param_file')],
    # )
    # ld.add_action(perception_param)
    # ld.add_action(perception_node)

    # #################### Planner Node #####################
    # planner_pkg_prefix = get_package_share_directory('planner')
    # planner_param_file = os.path.join(
    #     planner_pkg_prefix, 'config', 'params.yaml')
    
    # planner_param = DeclareLaunchArgument(
    #     'planner_param_file',
    #     default_value=planner_param_file,
    #     description='Path to config file for planner node'
    # )
    # planner_node = Node(
    #     package='planner',
    #     name='planner_node',
    #     executable='planner_node',
    #     parameters=[LaunchConfiguration('planner_param_file')],
    # )
    # ld.add_action(planner_param)
    # ld.add_action(planner_node)

    # #################### Control Node #####################
    control_pkg_prefix = get_package_share_directory('control')
    control_param_file = os.path.join(
        control_pkg_prefix, 'config', 'params.yaml')
    
    control_param = DeclareLaunchArgument(
        'control_param_file',
        default_value=control_param_file,
        description='Path to config file for control node'
    )
    control_node = Node(
        package='control',
        name='control_node',
        executable='control_node',
        parameters=[LaunchConfiguration('control_param_file')],
    )
    ld.add_action(control_param)
    ld.add_action(control_node)

    return ld
