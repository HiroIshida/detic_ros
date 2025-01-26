FROM nvcr.io/nvidia/l4t-pytorch:r35.1.0-pth1.11-py3
ENV DEBIAN_FRONTEND=noninteractive

RUN echo 'Etc/UTC' > /etc/timezone && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN \
  useradd user && \
  echo "user ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/user && \
  chmod 0440 /etc/sudoers.d/user && \
  mkdir -p /home/user && \
  chown user:user /home/user && \
  chsh -s /bin/bash user

RUN echo 'root:root' | chpasswd
RUN echo 'user:user' | chpasswd

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    build-essential \
    sudo \
    git \
    dirmngr \
    gnupg2 \
    curl \
    lsb-release \
    wget \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# setup keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

# Remove OpenCV built with CUDA by NVIDIA. It conflicts with original OpenCV deb
RUN apt purge opencv-* -y

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    ros-noetic-ros-base=1.5.0-1* \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    ros-noetic-jsk-tools \
    ros-noetic-image-transport-plugins \
    python3-osrf-pycommon \
    python3-catkin-tools \
    python3-wstool \
    ros-noetic-jsk-pcl-ros \
    ros-noetic-jsk-pcl-ros-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/user

USER user
CMD /bin/bash
SHELL ["/bin/bash", "-c"]

########################################
########### WORKSPACE BUILD ############
########################################
# Installing catkin package
RUN mkdir -p ~/detic_ws/src
RUN sudo rosdep init && rosdep update

# Build detectron2 from source. The aarch64 version is not released
RUN cd /tmp &&\
    git clone -b v0.6 https://github.com/facebookresearch/detectron2 &&\
    pip3 install --no-cache-dir -e detectron2

RUN mkdir -p ~/detic_ws/src/detic_ros/node_script
COPY --chown=user requirements.txt /home/user/detic_ws/src/detic_ros
COPY --chown=user prepare.sh /home/user/detic_ws/src/detic_ros
RUN cd ~/detic_ws/src/detic_ros && ./prepare.sh &&\
    rm -rf ~/.cache/pip

COPY --chown=user . /home/user/detic_ws/src/detic_ros
RUN cd ~/detic_ws/src/detic_ros && git submodule update --init --recursive
RUN cd ~/detic_ws/src &&\
    source /opt/ros/noetic/setup.bash &&\
    wstool init &&\
    wstool merge detic_ros/rosinstall.noetic &&\
    wstool update &&\
    sudo apt update &&\
    rosdep install --from-paths . --ignore-src -y -r &&\
    source /opt/ros/noetic/setup.bash &&\
    rosdep install --from-paths . -i -r -y &&\
    sudo rm -rf /var/lib/apt/lists/* &&\
    cd ~/detic_ws && catkin init && catkin build

# to avoid conflcit when mounting
RUN rm -rf ~/detic_ws/src/detic_ros/launch

########################################
########### ENV VARIABLE STUFF #########
########################################
RUN touch ~/.bashrc
RUN echo "source ~/detic_ws/devel/setup.bash" >> ~/.bashrc
RUN echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

RUN sudo usermod -aG video user

CMD ["bash"]
