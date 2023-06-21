FROM althack/ros2:foxy-cuda-gazebo-nvidia

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-foxy-gazebo-*\
   ros-foxy-cartographer \
   ros-foxy-cartographer-ros \
   ros-foxy-navigation2 \
   ros-foxy-nav2-bringup \
   ros-foxy-dynamixel-sdk \
   ros-foxy-turtlebot3-msgs \
   ros-foxy-turtlebot3 \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc

ENV TURTLEBOT3_MODEL=burger

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN apt update && apt-get install -y ros-foxy-rosbridge-suite 
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

