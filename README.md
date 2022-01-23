## detic_ros
ROS package for [Detic](https://github.com/facebookresearch/Detic). 

This package is still in under active-development. [Here](https://github.com/HiroIshida/detic_ros/issues/2) is the current TODO list.

![image](https://drive.google.com/uc?export=view&id=1aiWK51VL9pQvEKABpodRG7CkJRcjZodw)


## Howto use
Build docker image
```bash
git clone https://github.com/HiroIshida/detic_ros.git
cd detic_ros
docker build -t detic_ros .
```

Example for running node on pr1040 network:
```bash
docker run --rm --net=host -it --gpus 1 detic_ros:latest \
    /bin/bash -i -c \
    'source ~/.bashrc; \
    rossetip; rossetmaster pr1040; \
    roslaunch detic_ros sample.launch input_image:=/kinect_head/rgb/half/image_rect_color'
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

