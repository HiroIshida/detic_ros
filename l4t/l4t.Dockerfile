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

# Copy repository and install system dependencies
RUN rosdep init
RUN mkdir -p /catkin_ws/src/detic_ros
COPY package.xml /catkin_ws/src/detic_ros
RUN cd /catkin_ws/src/detic_ros &&\
    apt update &&\
    rosdep update --include-eol-distros &&\
    rosdep install --rosdistro=noetic -iqry --from-paths /catkin_ws/src &&\
    rm -rf /var/lib/apt/lists/*

# Build
COPY . /catkin_ws/src/detic_ros
RUN cd /catkin_ws &&\
    /opt/ros/noetic/env.sh catkin build detic_ros --cmake-args -DUSE_VIRTUALENV=OFF

RUN python3 -m venv /venv --system-site-packages

# Upgrade pip-setup related packages to make grpcio install faster. Without this line, the grpcio would be compiled from source and it takes so much time.
RUN /venv/bin/pip3 install --upgrade pip 'setuptools<=68.2.2' wheel

# Build detectron2 fron source because of not being released in aarch64
RUN /venv/bin/pip3 install git+https://github.com/facebookresearch/detectron2@v0.6 --no-cache-dir &&\
    rm -rf ~/.cache/pip

RUN /venv/bin/pip3 install -r /catkin_ws/src/detic_ros/l4t/requirements_l4t.txt &&\
    rm -rf ~/.cache/pip

# Pre-download CLIP embeddings to avoid downloading it on runtime
RUN /venv/bin/python3 -c "import clip; clip.load('ViT-B/32', device='cpu')"

ENTRYPOINT ["/catkin_ws/src/detic_ros/l4t/entrypoint_l4t.sh"]
