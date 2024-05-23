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
    rviz_file = "lidar_bot.rviz"

    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, "worlds", world_file_name)
    urdf_file = os.path.join(pkg_path, "description", robot_file)
    rviz_config=os.path.join(pkg_path,"config",rviz_file)

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')


    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description={'robot_description':doc.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )


    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity','cap','-file',urdf_file]
    )

    # rviz_start = ExecuteProcess(
    #     cmd=["ros2","run","rviz2","rviz2","-d",rviz_config],output="screen"
    # )

    rviz_start = ExecuteProcess(
        cmd=["ros2", "run", "rviz2", "rviz2", "-d", rviz_config], output="screen"
    )

    return LaunchDescription(
        [
             TimerAction(
                period=3.0,
                actions=[rviz_start]
            ),
            start_gazebo_server_cmd,
            start_gazebo_client_cmd,
            spawn_entity,
            robot_state_publisher_node,
            joint_state_publisher
         
            
        ]
    )

