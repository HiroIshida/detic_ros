## detic_ros
ROS package for [Detic](https://github.com/facebookresearch/Detic)

![image](https://drive.google.com/uc?export=view&id=1aiWK51VL9pQvEKABpodRG7CkJRcjZodw)

## Installation
Please install torch compatible with your cuda version beforehand:
```bash
pip3 install torch==1.9.0+cu111 torchvision==0.10.0+cu111 torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html # 
```
see [official installation guide](https://pytorch.org/get-started/previous-versions/) for detail.

Then, build ros package: (if you use kinetic/melodic, configure you workspace using python3)
```bash
mkdir -p ~/detic_ws/src
cd ~/detic_ws/src
git clone https://github.com/HiroIshida/detic_ros.git
cd detic_ros
pip3 install -r requirements.txt
./prepare.sh
source /opt/ros/noetic/setup.bash 
rosdep install from-paths . -i -r -y
cd ~/detic_ws && catkin build
```

## Run using PR2 topics (jsk lab only)
```bash
roslaunch detic_ros sample.launch
```