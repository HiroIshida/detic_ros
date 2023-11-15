# detic_ros [![rostest](https://github.com/HiroIshida/detic_ros/actions/workflows/docker_build.yml/badge.svg)](https://github.com/HiroIshida/detic_ros/actions/workflows/docker_build.yml) [![peripheral](https://github.com/HiroIshida/detic_ros/actions/workflows/peripheral.yml/badge.svg)](https://github.com/HiroIshida/detic_ros/actions/workflows/peripheral.yml)

ROS package for [Detic](https://github.com/facebookresearch/Detic). Run on both CPU and GPU, GPU is way performant, but work fine also with CPU (take few seconds to process single image).

![image](https://drive.google.com/uc?export=view&id=1aiWK51VL9pQvEKABpodRG7CkJRcjZodw)

example of custom vocabulary. Left: default (lvis), Right: custom ('bottle,shoe')

<img src='https://user-images.githubusercontent.com/67531577/161755658-7a0eeff2-0559-4320-a438-0904a0e3c515.png' width=25%> <img src='https://user-images.githubusercontent.com/67531577/161755524-b39ae38b-797a-4a57-9288-a569403c5909.png' width=25%>

example of three dimensional pose recognition for cups, bottles, and bottle caps.

<img src='https://user-images.githubusercontent.com/20625381/199499891-c5b66103-0de1-4619-ad64-0d1571efba92.png' width=50%>

## step1: build docker container
*Ofcourse you can build this pacakge on your workspace and launch as normal ros package. But for those using CUDA, the following docker based approach might be safer and easy.*


Prerequsite: You need to preinstall nvidia-container-toolkit beforehand. see (https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

Build docker image
```bash
git clone https://github.com/HiroIshida/detic_ros.git
cd detic_ros
docker build -t detic_ros .
```
## step2: launch Detic-segmentor node
Example for running node on pr1040 (**please replace `pr1040` by you robot hostname or `localhost`**):
```bash
python3 run_container.py -host pr1040 -mount ./launch -name sample.launch \
    out_debug_img:=true \
    out_debug_segimg:=false \
    compressed:=false \
    device:=auto \
    input_image:=/kinect_head/rgb/image_color
```
The minimum necessary argument of `run_container.py` is `host`, `mount` and `name`:
- host: host name or IP address
- mount: launch file or launch file's directory path that will be mounted inside the container. In this example, launch file directory of this repository is mounted.
- name: launch file name that will be searched from mounted file or directory
Also, you can specify launch args as the roslaunch command (e.g. `out_debug_img:=true`). This launch args must come after the above three args.

Another example for running three dimensional object pose detection using point cloud filtered by segmentation.
```bash
python3 run_container.py -host pr1040 -mount ./launch -name sample_detection.launch \
    debug:=true \
    vocabulary:=custom \
    custom_vocabulary:=bottle,cup
```
Or `rosrun detic_ros run_container.py` if you catkin build this package on the hosting computer side.
As in this example, by putting required sub-launch files inside the directory that will be mounted on, you can combine many node inside the container.

### Note:
- On custom vocabulary: if you want to limit the detected instances by custom vocabulary, please set launch args to `vocabulary:='custom' custom_vocabulary:='bottle,shoe'`. 
- On model types: Detic is trained in different model types. In this repository you can try out all of the [real-time models](https://github.com/facebookresearch/Detic/blob/main/docs/MODEL_ZOO.md#real-time-models) using the `model_type` parameter.
- On real-time performance: For higher recognition frequencies turn off all debug info, run on GPU, decompress topics locally, use smaller models (*e.g.* `res50`), and avoid having too many classes in the frame (by *e.g.* setting a custom vocabulary or higher confidence thresholds). The `sample_detection.launch` with default parameters handles all of this, yielding object bounding boxes at **around 10Hz**.

## step3a (Subscribe from node in step3 and do anything you want)
Example for using the published topic from the node above is [masked_image_publisher.py](./example/masked_image_publisher.py). This will be helpful for understanding how to apply `SegmentationInfo` message to a image. The [test file](/test/test_node.test) for this example also might be helpful.

## step3b (Service call)
See definition of [`srv/DeticSeg.srv`](srv/DeticSeg.srv)

## ROS node information
- `~input_image` (`sensor_msgs/Image`)
  - Input image
- `~debug_image` (`sensor_msgs/Image`)
  - debug image 
- `~debug_segmentation_image` (`sensor_msgs/Image` with `32SC1` encoding)
  - Say detected class number is 14, `~segmentation_image` in grayscale image is almost completely dark and not good for debugging. Therefore this topic scale the value to [0 ~ 255] so that grayscale image is human-friendly.
- `~segmentation_info` (`detic_ros/SegmentationInfo`)
  - Published when `use_jsk_msgs` is false. Includes the class name list, confidence score list and segmentation image with `32SC1` encoding. The image is filled by 0 and positive integers indicating segmented object number. These indexes correspond to one plus those of class name list and confidence score list. For example, an image value of 2 corresponds to the second (index=1) item in the class name and score list. Note that the image value of 0 is always reserved for the 'background' instance.
- `~segmentation` (`sensor_msgs/Image`)
  - Published when `use_jsk_msgs` is true. Includes the segmentation image with `32SC1` encoding.
- `~detected_classes` (`jsk_recognition_msgs/LabelArray`)
  - Published when `use_jsk_msgs` is true. Includes the names and ids of the detected objects. In the same order as `~score`.
- `~score` (`jsk_recognition_msgs/VectorArray`)
  - Published when `use_jsk_msgs` is true. Includes the confidence score of the detected objects. In the same order as `~detected_classes`.

As for rosparam, see [node_cofig.py](./node_script/node_config.py).

## Running without roscore to batch processing a bag file
```bash
rosrun detic_ros batch_processor.py path/to/bagfile
```
See [source code](/node_script/batch_processor.py) for the options.
