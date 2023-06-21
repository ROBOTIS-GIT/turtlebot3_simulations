FROM althack/ros2:foxy-gazebo-nvidia

ENV ROS_DISTRO ${ROS_DISTRO}

# install utils
RUN apt-get update && DEBIAN_FRONTEND=noninteractive  && apt-get install -y  --no-install-recommends\
    autoconf \
    automake \
    libtool \
    make \
    g++ \
    unzip \
    libprotobuf-dev \
    wget \
    openssh-server \
    curl \
    gnupg \
    git \
    build-essential \
    cmake \
    gdb-multiarch \
    default-jre \
    python3 \
    python3-setuptools \
    python3-pip \
    python3-venv \
    ros-${ROS_DISTRO}-rosbridge-suite \    
    && rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ros-$ROS_DISTRO-rosbag2 \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-ros2bag \
    ros-$ROS_DISTRO-rosbag2-transport \
    && rm -rf /var/lib/apt/lists/* 

RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-gazebo-*\
   ros-${ROS_DISTRO}-cartographer \
   ros-${ROS_DISTRO}-cartographer-ros \
   ros-${ROS_DISTRO}-navigation2 \
   ros-${ROS_DISTRO}-nav2-bringup \
   ros-${ROS_DISTRO}-dynamixel-sdk \
   ros-${ROS_DISTRO}-turtlebot3-msgs \
   ros-${ROS_DISTRO}-turtlebot3 \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

RUN  pip install mcap-ros2-support citros

RUN echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc

WORKDIR /workspaces/turtlebot3

COPY src/ src/
COPY ros2_entrypoint.sh ros2_entrypoint.sh

RUN . /opt/ros/${ROS_DISTRO}/setup.sh
RUN colcon build --symlink-install 

ENV TURTLEBOT3_MODEL=waffle
 
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /workspaces/turtlebot3/install/local_setup.bash" >> ~/.bashrc  

RUN apt-get update && apt-get -y dist-upgrade --no-install-recommends   

RUN chmod +x ros2_entrypoint.sh
ENTRYPOINT ["/workspaces/turtlebot3/ros2_entrypoint.sh"]

CMD ["bash"]