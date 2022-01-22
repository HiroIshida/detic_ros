## detic_ros
ROS package for [Detic](https://github.com/facebookresearch/Detic). 

This package is still in under active-development. [Here](https://github.com/HiroIshida/detic_ros/issues/2) is the current TODO list.

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

## ROS node information
- `~input_image` (`sensor_msgs/Image`)
  - Input image
- `~debug_image` (`sensor_msgs/Image`)
  - debug image 
- `~segmentation_image` (`sensor_msgs/Image` with `8UC1` encoding)
  - Segmentation image. Suppose detected class number is 14, image is filled with 0~14 uint8 values. Note that 0 means background label.
- `~debug_segmentation_image` (`sensor_msgs/Image` with `8UC1` encoding)
  - Say detected class number is 14, `~segmentation_image` in grayscale image is almost completely dark and not good for debugging. Therefore this topic scale the value to [0 ~ 255] so that grayscale image is human-friendly.
- `~segmentation_info` (`detic_ros/SegmentationInfo`)
  - class name list and confidence score list corresponding to `~segmentation_image`

As for rosparam, see [node_cofig.py](./node_script/node_config.py).
