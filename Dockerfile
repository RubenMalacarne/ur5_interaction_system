FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    software-properties-common && \
    add-apt-repository universe && \
    apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    nano \
    git \
    wget \
    vim \
    tar xz-utils \
    libx11-6 libxcb1 libxau6 libgl1-mesa-dev \
    xvfb dbus-x11 x11-utils libxkbcommon-x11-0 \
    libavcodec-dev libavformat-dev libswscale-dev \
    python3-venv libraw1394-11 libmpfr6 libusb-1.0-0 \
    xsltproc \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-moveit* \
    ros-humble-gazebo-* \
    ros-humble-ros-gz* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    libxcb-xinerama0 \  
    && apt-get clean && rm -rf /var/lib/apt/lists/*



# Install Python packages
RUN pip3 install pyzmq cbor2 \
    pyserial \
    flask \
    flask-ask-sdk \
    ask-sdk \
    xmlschema

# ROS workspace
WORKDIR /ros2_ws
COPY . /ros2_ws/src_CR
# install first bash
# RUN cd /ros2_ws/src_CR && chmod +x installation_docker.sh
# RUN cd /ros2_ws/src_CR && ./installation_docker.sh

# install third bash
RUN cd /ros2_ws/src_CR && chmod +x download_coppelia.sh
RUN mkdir -p /ros2_ws/src_CR/download
RUN cd /ros2_ws/src_CR && ./download_coppelia.sh -d

RUN cp /ros2_ws/src_CR/download/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04.tar.xz /opt/
RUN tar -xf /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04.tar.xz -C /opt && \
    rm /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04.tar.xz
RUN rm -rf /ros2_ws/src_CR/download

ENV COPPELIASIM_ROOT_DIR=/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
ENV PATH=$COPPELIASIM_ROOT_DIR:$PATH
ENV LD_LIBRARY_PATH=$COPPELIASIM_ROOT_DIR:$LD_LIBRARY_PATH
ENV QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms
ENV QT_QPA_PLATFORM=xcb

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

CMD ["bash"]
