#!/usr/bin/env python3

import launch
import os
import sys

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
        LaunchConfiguration,
        IfElseSubstitution,
        PythonExpression,
        PathJoinSubstitution,
        EnvironmentVariable,
        )

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_autostart"

    this_pkg_path = get_package_share_directory(pkg_name)
    namespace='automatic_start'

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
            )

    # #} end of custom_config

    # #{ env-based params

    run_type=os.getenv('RUN_TYPE', "realworld")

    if run_type == "simulation":
        simulation = True
    else:
        simulation = False

    # #} end of env-based params

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    ld.add_action(ComposableNodeContainer(

        namespace=uav_name,
        name=namespace+'_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],

        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],

        composable_node_descriptions=[

            ComposableNode(

                package=pkg_name,
                plugin='mrs_uav_autostart::automatic_start::AutomaticStart',
                namespace=uav_name,
                name='automatic_start',

                parameters=[
                    {"uav_name": uav_name},
                    {"simulation": simulation},
                    {"enable_profiler": False},
                    {"use_sim_time": use_sim_time},
                    {'config_private': this_pkg_path + '/config/private/automatic_start.yaml'},
                    {'config_public': this_pkg_path + '/config/public/automatic_start.yaml'},
                    {'custom_config': custom_config},
                    ],

                remappings=[
                    # subscribers
                    ("~/hw_api_status_in", "hw_api/status"),
                    ("~/hw_api_capabilities_in", "hw_api/capabilities"),
                    ("~/control_manager_diagnostics_in", "control_manager/diagnostics"),
                    ("~/safety_area_manager_diagnostics_in", "safety_area_manager/diagnostics"),
                    ("~/uav_manager_diagnostics_in", "uav_manager/diagnostics"),
                    ("~/gazebo_spawner_diagnostics_in", "/mrs_drone_spawner/diagnostics"),
                    ("~/estimation_diag_in", "estimation_manager/diagnostics"),
                    ("~/distance_sensor_in", "hw_api/distance_sensor"),
                    ("~/imu_in", "hw_api/imu"),
                    # publishers
                    ("~/can_takeoff_out", "~/can_takeoff"),
                    # services out
                    ("~/takeoff_out", "uav_manager/takeoff"),
                    ("~/toggle_control_output_out", "control_manager/toggle_output"),
                    ("~/arm_out", "hw_api/arming"),
                    ("~/validate_reference_out", "control_manager/validate_reference_2d"),
                ],
            )

        ],

    ))

    return ld
