^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot3_simulations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2021-01-07)
------------------
* fix init() in turtlebot3_drive.cpp
* Apply low poly models
* Noetic release
* Contributors: Sean Yen, Will Son

1.3.0 (2020-09-29)
------------------
* fix ROS Assert issue when debugging (#124)
* added TurtleBot3 Autorace 2020 (#108)
* added turtlebot3_description to dependency list (#104)
* removed *nix path separator (#92)
* removed unnecessary gazebo plugin_path (#78)
* Contributors: Sean Yen, Ashe Kim, Mikael Arguedas, Ben Wolsieffer

1.2.0 (2019-01-22)
------------------
* move out the init() from ROS_ASSERT `#68 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/68>`_
* moved <scene> into <world> `#65 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/65>`_
* modified ML stage
* delete unused param
* update algorithm and modified variable more clearly
* Contributors: Darby Lim, Gilbert, Louise Poubel, Pyo

1.1.0 (2018-07-20)
------------------
* added TurtleBot3 Waffle Pi
* modified uri path
* modified autorace
* delete remap
* Contributors: Darby Lim, Gilbert, Pyo

1.0.2 (2018-06-01)
------------------
* added mission.launch modified model.sdf
* deleted turtlebot3's gazebo plugins
* modified autorace gazebo
* merged pull request `#53 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/53>`_ `#52 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/52>`_ `#51 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/51>`_ `#50 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/50>`_ `#49 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/49>`_
* Contributors: Gilbert, Darby Lim, Pyo

1.0.1 (2018-05-30)
------------------
* resolving dependency issues:
  http://build.ros.org/job/Kbin_dj_dJ64__turtlebot3_gazebo__debian_jessie_amd64__binary/2/
* Contributors: Pyo

1.0.0 (2018-05-29)
------------------
* added world for turtlebot3_autorace
* added world for turtlebot3_machine_learning
* merged pull request `#46 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/46>`_ from AuTURBO/develop
  add turtlebot3_autorace world'
* merged pull request `#48 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/48>`_ `#47 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/47>`_ `#44 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/44>`_ `#42 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/42>`_ `#41 <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/41>`_
* Contributors: Darby Lim, Gilbert, hyunoklee, Pyo

0.2.4 (2018-03-14)
------------------
* solved DuplicateVersionsException error
* Contributors: Pyo

0.2.3 (2018-03-14)
------------------
* solved DuplicateVersionsException error
* Contributors: Pyo

0.2.2 (2018-03-14)
------------------
* added line feed into metapackage
* Contributors: Pyo

0.2.1 (2018-03-14)
------------------
* added worlds for gazebo and turtlebot3
* deleted turtlebot3_gazebo_plugin and merged into turtlebot3_gazebo_ros package
* Contributors: Darby Lim

0.2.0 (2018-03-13)
------------------
* added TurtleBot3 Waffle Pi
* added slam with multiple tb3
* added multi example
* added turtlebot3_house
* added turtlebot3_house
* added msg function
* modified gazebo plugin
* modified tb3 control
* modified sensor param
* modified camera position
* modified image_listener
* modified cmake file
* modified spwn model name
* modified multi slam param
* modified camera position
* modified folder name
* Contributors: Darby Lim

0.1.7 (2017-08-16)
------------------
* renamed missed the install rule (worlds -> models)
* Contributors: Darby Lim, Tully Foote

0.1.6 (2017-08-14)
------------------
* modified folder name and model path
* updated rviz and add static tf publisher for depth camera
* Contributors: Darby Lim

0.1.5 (2017-06-09)
------------------
* modified make files for dependencies
* updated turtlebot3 sim
* updated world config
* Contributors: Darby Lim

0.1.4 (2017-05-23)
------------------
* added as new meta-packages and version update (0.1.4)
* Contributors: Darby Lim, Pyo
