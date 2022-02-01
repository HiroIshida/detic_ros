## detic_ros
ROS package for [Detic](https://github.com/facebookresearch/Detic). 

This package is still in under active-development. [Here](https://github.com/HiroIshida/detic_ros/issues/2) is the current TODO list.

![image](https://drive.google.com/uc?export=view&id=1aiWK51VL9pQvEKABpodRG7CkJRcjZodw)


## Howto use
#### Prerequisite
`nvidia-container-toolkit` is required.
```
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```
#### Building docker image
```bash
git clone https://github.com/HiroIshida/detic_ros.git
cd detic_ros
docker build -t detic_ros .
```

#### step1 (launch Detic-segmentor node)
Example for running node on pr1040 network:
```bash
docker run --rm --net=host -it --gpus 1 detic_ros:latest \
    /bin/bash -i -c \
    'source ~/.bashrc; \
    rossetip; rossetmaster pr1040; \
    roslaunch detic_ros sample.launch \
    out_debug_img:=true \
    out_debug_segimg:=false \
    compressed:=false \
    input_image:=/kinect_head/rgb/image_color'
```
Change the `pr1040` part and `/kinect_head/rgb/image_color` in command above by your custom host name and an image topic. If compressed image (e.g. `/kinect_head/rgb/image_color/compressed`) corresponding to the specified `input_image` is also published, by setting `compressed:=true`, you can reduce the topic pub-sub latency.

#### step2 (Subscribe from node in step1 and do something)
Example for using the published topic from the node above is [masked_image_publisher.py](./example/masked_image_publisher.py). By using subscribed segmentation image and segmentation info and, this node converts a subscribed rgb image into a masked rgb image.

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
  - class name list and confidence score list corresponding to `~segmentation_image`. Note that `score` of `background` class is always 1.0

As for rosparam, see [node_cofig.py](./node_script/node_config.py).

