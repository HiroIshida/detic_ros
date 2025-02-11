FROM nvcr.io/nvidia/l4t-pytorch:r35.1.0-pth1.11-py3

ENV DEBIAN_FRONTEND=noninteractive

# Remove OpenCV built with CUDA by NVIDIA. It conflicts with original OpenCV deb
RUN apt purge opencv-* -y

# Install ROS
RUN apt update && apt install -y -qq curl &&\
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list' &&\
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - &&\
    apt update && apt install -y -qq \
    python3-catkin-tools python3-rosdep ros-noetic-desktop-full ros-noetic-jsk-recognition &&\
    rm -rf /var/lib/apt/lists/*

# Upgrade pip-setup related packages to make grpcio install faster. Without this line, the grpcio would be compiled from source and it takes so much time.
RUN pip3 install --upgrade pip 'setuptools<=68.2.2' wheel && rm -rf ~/.cache/pip

# Build detectron2 from source. The aarch64 version is not released
RUN cd /tmp &&\
    git clone --depth 1 -b v0.6 https://github.com/facebookresearch/detectron2 &&\
    pip3 install --no-cache-dir -e detectron2 &&\
    rm -rf ~/.cache/pip

# Copy repository and install system dependencies
RUN rosdep init
RUN mkdir -p /catkin_ws/src/detic_ros
COPY package.xml /catkin_ws/src/detic_ros
RUN cd /catkin_ws/src/detic_ros &&\
    apt update &&\
    rosdep update &&\
    rosdep install --rosdistro=noetic -iqry --from-paths /catkin_ws/src &&\
    rm -rf /var/lib/apt/lists/*

# Build
COPY . /catkin_ws/src/detic_ros
RUN sed -i '1,3d' /catkin_ws/src/detic_ros/requirements_without_torch.txt
RUN cd /catkin_ws &&\
    /opt/ros/noetic/env.sh catkin build detic_ros --cmake-args -DDETIC_ROS_INSTALL_TORCH=OFF
