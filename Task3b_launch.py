#!/usr/bin/python3
# -*- coding: utf-8 -*-


import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory path
    package_dir = get_package_share_directory('ur_description')  # Replace with your package name

    # Paths to the scripts and config file within the package directory
    script2_path = os.path.join(package_dir, 'scripts', 'ebot_nav2_cmd_task4b.py')
    script3_path = os.path.join(package_dir, 'scripts', 'ebot_docking_service_task4b.py')
    script1_path = os.path.join(package_dir,'scripts', 'aruco_detection_3B.py')
    script4_path = os.path.join(package_dir,'scripts', 'Pick_and_place_box_3B.py')
    # Example path to a config file in a 'config' directory

    # Command to execute your first script
    navigation_command = ExecuteProcess(
        cmd=['python3', script1_path]
    )

    # Command to execute your second script
    docking_command = ExecuteProcess(
        cmd=['python3', script2_path]
    )

    aruco_detection_command = ExecuteProcess(
        cmd=['python3', script3_path]
    )

    Box_pick_and_place_command = ExecuteProcess(
        cmd = ['python3' , script4_path]
    )
    
    script1_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=navigation_command,
            on_exit=[aruco_detection_command, TimerAction(period=5.0, actions=[Box_pick_and_place_command])]  # Launch script3 after script1 exits, then start timer for script4
        )
    )


    return LaunchDescription([
    aruco_detection_command,
    navigation_command,
    Box_pick_and_place_command,
    docking_command
    ])
