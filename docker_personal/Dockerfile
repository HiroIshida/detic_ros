FROM nvidia/cuda:11.0-devel-ubuntu20.04

RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*


RUN apt update 

# install minimum tools:
RUN apt install -y build-essential sudo git vim tmux openssh-server net-tools

RUN \
  useradd user && \
  echo "user ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/user && \
  chmod 0440 /etc/sudoers.d/user && \
  mkdir -p /home/user && \
  chown user:user /home/user && \
  chsh -s /bin/bash user

RUN echo 'root:root' | chpasswd
RUN echo 'user:user' | chpasswd

# set up ssh
# https://github.com/IMC3ofC/db2express-c.docker/issues/12
RUN mkdir /var/run/sshd
RUN sed -i 's/\#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config && echo Port 2233 >> /etc/ssh/sshd_config
EXPOSE 2233

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install python3-osrf-pycommon python3-catkin-tools -y
RUN apt update && apt install ros-noetic-jsk-tools -y

WORKDIR /home/user
USER user
CMD /bin/bash
SHELL ["/bin/bash", "-c"]

#RUN sudo apt install python3-pip -y 
#RUN pip3 install torch torchvision
#RUN git clone https://github.com/facebookresearch/detectron2.git
#RUN cd detectron2 && pip3 install . --user
#
#RUN source /opt/ros/noetic/setup.bash
#RUN mkdir -p ~/detic_ws/src
#RUN cd ~/detic_ws/src && git clone https://github.com/HiroIshida/detic_ros.git
#RUN cd ~/detic_ws/src/detic_ros && git submodule update --init --recursive
#RUN cd ~/detic_ws/src/detic_ros/Detic && pip3 install -r requirements.txt 
#RUN cd ~/detic_ws/src/detic_ros && ./prepare_dataset.sh
#
#RUN source /opt/ros/noetic/setup.bash && cd ~/detic_ws && catkin build

# setup dotfiles (also source)
RUN git clone https://github.com/HiroIshida/dotfiles2.git
RUN cd dotfiles2 && ./install --light && exec bash
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bash/user_specific.sh
RUN echo "source ~/detic_ws/devel/setup.bash" >> ~/.bash/user_specific.sh
RUN echo 'export PATH=\"$PATH:$HOME/.local/bin"' >> ~/.bash/user_specific.sh 
