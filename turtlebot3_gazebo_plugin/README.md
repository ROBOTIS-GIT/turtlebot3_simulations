# Install library for gazebo7 development

$ sudo apt-get install libgazebo7-dev

# Add gazebo plugin path

$ export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${turtlebot3_gazebo_plugin}/build

$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${turtlebot3_gazebo_plugin}/models

# Make and Build

$ cd ${turtlebot3_gazebo_plugin}/build
$ cmake ..
$ make

# Excute

$ cd ${turtlebot3_gazebo_plugin}
$ gazebo worlds/turtlebot3_${TB3_MODEL}.world

  - TB3_MODEL = `burger`, `waffle`, `waffle_pi`

# Teleoperation with keyboard

w - set linear velocity up 
x - set linear velocity down
d - set angular velocity up
a - set angular velocity down
s - set all velocity to zero

# Topic subscribe command

## Show all topic
$ gz topic -l

## Subscribe scan data
$ gz topic -e /gazebo/default/user/turtlebot3_${TB3_MODEL}/lidar/hls_lfcd_lds/scan

## Subscribe image data

**Waffle**

$ gz topic -e /gazebo/default/user/turtlebot3_waffle/image/intel_realsense_r200/image

or

**Waffle Pi**

$ gz topic -e /gazebo/default/user/turtlebot3_waffle_pi/image/raspberry_pi_cam/image

## Excute listener

$ cd ${turtlebot3_gazebo_plugin}/build
$ ./lidar_listener ${TB3_MODEL}

or

$ cd ${turtlebot3_gazebo_plugin}/build
$ ./image_listener ${TB3_MODEL}


[Gazebo API](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/index.html)

[How to contribute model](http://gazebosim.org/tutorials?tut=model_contrib&cat=build_robot)
[How to make model](http://gazebosim.org/tutorials?tut=build_model&cat=build_robot)

[Tutorial for making Hello World plugin](http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin)
[Tutorial for making model plugin](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5)
[Tutorial for making sensor plugin](http://gazebosim.org/tutorials?tut=contact_sensor)

[Tutorial for topic subscription](http://gazebosim.org/tutorials?tut=topics_subscribed)

[Example of Wide-Angle Camera](http://gazebosim.org/tutorials?tut=wide_angle_camera&branch=wideanglecamera)
