# ros2_nav

## gazebo start & spawn robot

    gazebo --verbose -s libgazebo_ros_factory.so
    ros2 run gazebo_ros spawn_entity.py -entity  skidbot -file ./src/my_robot/description/skidbot.urdf
