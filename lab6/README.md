# МОДУЛЬ 6
```
$ cd ~/ros2_ws && colcon build --packages-select robot_description robot_bringup robot_app robot && source install/setup.bash
```

## задание 1-3:
```
$ LIBGL_ALWAYS_SOFTWARE=1 ros2 launch robot_bringup robot_lidar.launch.py
```

Gazebo:
    - search: teleop  
    - model/robot/cmd_vel

```
$ ros2 topic echo /imu
```

## задание 4
```
$ LIBGL_ALWAYS_SOFTWARE=1 ros2 launch robot_lidar robot_lidar.launch.py
```

## задание 5
терминал 1:
```
$ LIBGL_ALWAYS_SOFTWARE=1 ros2 launch robot_depth robot_depth.launch.py
```
