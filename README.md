# ros2_nav

## gazebo start & spawn robot

    gazebo --verbose -s libgazebo_ros_factory.so
    ros2 run gazebo_ros spawn_entity.py -entity  skidbot -file ./src/my_robot/description/skidbot.urdf
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/skidbot

![image](https://github.com/Dong-geon24/ros2_nav/assets/166792318/0897fb3b-9ff8-4ef3-b7ac-f7375967b2f8)
