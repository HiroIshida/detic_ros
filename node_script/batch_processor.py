#!/usr/bin/env python3
import argparse
import os
import pickle
from typing import List, Optional

import numpy as np
import rosbag
import tqdm
from cv_bridge import CvBridge
from moviepy.editor import ImageSequenceClip
from sensor_msgs.msg import Image

from detic_ros.node_config import NodeConfig
from detic_ros.wrapper import DeticWrapper, InferenceRawResult


def bag_to_images(file_path: str, topic_name_extract: Optional[str] = None):
    bag = rosbag.Bag(file_path)
    image_list_: List[Image] = []

    for topic_name, msg, t in bag.read_messages():

        if topic_name_extract is None:
            if msg.__class__.__name__ == '_sensor_msgs__Image':
                image_list_.append(msg)
        else:
            if topic_name == topic_name_extract:
                image_list_.append(msg)

    bag.close()

    def deep_cast(msg):
        # must exist etter way ... but I don't have time
        # see:
        # https://github.com/ros/ros_comm/issues/769
        msg_out = Image()
        msg_out.header.seq = msg.header.seq
        msg_out.header.stamp = msg.header.stamp
        msg_out.header.frame_id = msg.header.frame_id
        msg_out.height = msg.height
        msg_out.width = msg.width
        msg_out.encoding = msg.encoding
        msg_out.is_bigendian = msg.is_bigendian
        msg_out.step = msg.step
        msg_out.data = msg.data
        return msg_out

    image_list = [deep_cast(msg_) for msg_ in image_list_]
    return image_list


def dump_result_as_pickle(
        results: List[InferenceRawResult],
        images: List[Image],
        output_file_name: str):

    result_dict = {'image': [], 'seginfo': [], 'debug_image': []}  # type: ignore
    for result, image in zip(results, images):
        seginfo = result.get_segmentation_info()
        debug_image = result.get_ros_debug_image()

        result_dict['image'].append(image)
        result_dict['seginfo'].append(seginfo)
        result_dict['debug_image'].append(debug_image)

    with open(output_file_name, 'wb') as f:
        pickle.dump(result_dict, f)


def dump_result_as_rosbag(
        input_bagfile_name: str,
        results: List[InferenceRawResult],
        output_file_name: str):

    bag_out = rosbag.Bag(output_file_name, 'w')

    bag_inp = rosbag.Bag(input_bagfile_name)
    for topic_name, msg, t in bag_inp.read_messages():
        bag_out.write(topic_name, msg, t)
    bag_inp.close()

    for result in results:
        seginfo = result.get_segmentation_info()
        debug_image = result.get_ros_debug_image()
        bag_out.write('/detic_segmentor/segmentation_info', seginfo, seginfo.header.stamp)
        bag_out.write('/detic_segmentor/debug_image', debug_image, debug_image.header.stamp)
    bag_out.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=str, help='input file path')
    parser.add_argument('-model', type=str, default='swin', help='model type')
    parser.add_argument('-topic', type=str, default='', help='topic name')
    parser.add_argument('-n', type=int, default=-1, help='number of image to be processed')
    parser.add_argument('-out', type=str, default='', help='out file path')
    parser.add_argument('-format', type=str, default='pkl', help='out file format')
    parser.add_argument('-th', type=float, default=0.5, help='confidence threshold')
    parser.add_argument('-device', type=str, default='auto', help='device name')

    args = parser.parse_args()
    input_file_path = args.input
    model_type = args.model
    topic_name = args.topic
    output_file_name = args.out
    output_format = args.format
    confidence_threshold = args.th
    device = args.device
    n = args.n

    assert device in ['cpu', 'cuda', 'auto']
    assert output_format in ['bag', 'pkl']

    rw, ext_input = os.path.splitext(input_file_path)
    assert ext_input == '.bag'

    if output_file_name == '':
        output_file_name = rw + '_segmented.' + output_format
    debug_file_name = rw + '_debug.gif'

    topic_name = None if topic_name == '' else topic_name
    image_list = bag_to_images(input_file_path, topic_name)

    image_list = image_list if n == -1 else image_list[:n]
    assert len(image_list) > 0
    print('{} images found'.format(len(image_list)))

    node_config = NodeConfig.from_args(model_type,
                                       False, True, False, False, False,
                                       confidence_threshold, device)
    detic_wrapper = DeticWrapper(node_config)
    results = [detic_wrapper.infer(image) for image in tqdm.tqdm(image_list)]

    if output_format == 'pkl':
        dump_result_as_pickle(results, image_list, output_file_name)
    elif output_format == 'bag':
        dump_result_as_rosbag(input_file_path, results, output_file_name)

    # dump debug gif image
    bridge = CvBridge()

    def convert(msg) -> np.ndarray:
        return bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    debug_images = [result.get_ros_debug_image() for result in results]
    images = list(map(convert, debug_images))
    clip = ImageSequenceClip(images, fps=20)
    clip.write_gif(debug_file_name, fps=20)
