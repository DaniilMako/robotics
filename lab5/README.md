# МОДУЛЬ 5

cd ~/ros2_ws && colcon build --packages-select robot_description robot_bringup robot_app robot && source install/setup.bash

## задание 2:
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch robot_bringup diff_drive.launch.py

## задание 3
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch robot_bringup diff_drive.launch.py

rviz:
    rqt_robot_steering: robot/cmd_vel

Gazebo:
    search: teleop
    model/robot/cmd_vel

## задание 4
>терминал 1:
$ LIBGL_ALWAYS_SOFTWARE=1 ros2 launch robot_bringup diff_drive.launch.py

>>>>терминал 2:
ros2 run robot cycle

## задание 5
>>>>терминал 1:
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch robot_bringup diff_drive.launch.py

>>>>терминал 2:
ros2 run robot move
