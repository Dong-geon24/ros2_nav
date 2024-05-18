import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    package_name = "my_robot"
    robot_file = "skidbot.urdf"
    world_file_name = "room_world"

    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, "worlds", world_file_name)
    urdf_file = os.path.join(pkg_path, "description", robot_file)

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    # # Start Gazebo server
    # start_gazebo_server_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    #     launch_arguments={'world': world_path}.items()
    # )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )
    print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
    print(world_file_name,world_path)

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity','skidbot','-file',urdf_file]
    )
    return LaunchDescription(
        [
            start_gazebo_server_cmd,
            start_gazebo_client_cmd,
            spawn_entity,
            
        ]
    )

