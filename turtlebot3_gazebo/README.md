# TurtleBot3
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/turtlebot3/logo_turtlebot3.png" width="300">

## How to run TB3 in Gazebo

### Setup

```
$ mkdir -p ~/turtlebot3_ws/src
$ cd ~/turtlebot3_ws/src
$ git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ git clone -b master https://github.com/ros2/cartographer.git
$ git clone -b master https://github.com/ros2/cartographer_ros.git
$ git clone https://github.com/ros2/pcl_conversions.git
$ sudo apt install -y \
    clang \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    ninja-build \
    python-sphinx \
    libpcl-conversions-dev \
    libpcl-dev \
```

- Install ceres-solver (http://ceres-solver.org/installation.html#linux)


- colcon build

```
$ cd ~/turtlebot3_ws && colcon build
```

### Run Gazebo

```
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
$ cd ~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds
$ gazebo --verbose turtlebot3_ros2_demo.world
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

$ ros2 run tf2_ros tf2_echo base_link right_wheel

$ ros2 run tf2_ros tf2_echo base_link left_wheel
```