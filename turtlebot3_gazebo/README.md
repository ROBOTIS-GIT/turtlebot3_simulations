# TurtleBot3
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/turtlebot3/logo_turtlebot3.png" width="300">

### Run Gazebo

```
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_overlay_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
$ cd ~/ros2_overlay_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds
$ gazebo --verbose turtlebot3_ros2_demo.world -s libgazebo_ros_init.so
``` 

### Run tf2_monitor

```
$ ros2 run tf2_ros tf2_monitor
```

### Run teleop node

```
$ ros2 run turtlebot3_teleop turtlebot3_teleop_key
```

### Run Rviz

```
$ ros2 launch turtlebot3_bringup turtlebot3_robot.launch.py
$ rviz2
```

### Useful Commands

- Try sending commands:

```
$ ros2 topic pub /tb3/cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}}' -1

$ ros2 topic pub /tb3/cmd_vel geometry_msgs/Twist '{angular: {z: 0.1}}' -1
```

- Try listening to odometry:

```
$ ros2 topic echo /demo/odom_demo
```

- Try listening to TF:

```
$ ros2 run tf2_ros tf2_echo odom base_link

$ ros2 run tf2_ros tf2_echo base_footprint wheel_right_wheel

$ ros2 run tf2_ros tf2_echo base_footprint wheel_left_wheel
```

### Run Cartographer

```
$ ros2 launch turtlebot3_cartographer turtlebot3_cartographer.py use_sim_time:=True
```