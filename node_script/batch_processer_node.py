#!/usr/bin/env python3
import argparse
import os
from typing import List, Optional
import pickle

import rosbag
import rospy
from sensor_msgs.msg import Image
from detic_ros.msg import SegmentationInfo
import tqdm

from node_config import NodeConfig
from wrapper import DeticWrapper


def bag_to_images(file_path: str, topic_name_extract: Optional[str] = None):
    bag = rosbag.Bag(file_path)
    image_list: List[Image] = []

    for topic_name, msg, t in bag.read_messages():
        # https://github.com/ros/ros_comm/issues/769

        if topic_name_extract is None:
            if msg.__class__.__name__ == '_sensor_msgs__Image':
                image_list.append(msg)
        else:
            if topic_name == topic_name_extract:
                image_list.append(msg)

    return image_list


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=str, help='input file path')
    parser.add_argument('-topic_name', type=str, default='', help='out file path')
    parser.add_argument('-out', type=str, default='', help='out file path')
    parser.add_argument('-th', type=float, default=0.5, help='confidence threshold')
    parser.add_argument('-device', type=str, default='auto', help='device name')

    args = parser.parse_args()
    input_file_path = args.input
    topic_name = args.topic_name
    output_file_path = args.out
    confidence_threshold = args.th
    device = args.device

    assert device in ['cpu', 'cuda', 'auto']

    rw, ext_input = os.path.splitext(input_file_path)
    assert ext_input == '.bag'

    if output_file_path == '':
        output_file_path = rw + '_segmented.pkl'

    if topic_name == '':
        topic_name = None

    image_list = bag_to_images(input_file_path)
    assert len(image_list) > 0
    print('{} images found'.format(len(image_list)))

    node_config = NodeConfig.from_args(True, False, False, confidence_threshold, device)
    detic_wrapper = DeticWrapper(node_config)

    result_dict = {'image': [], 'seginfo': [], 'debug_image': []}
    for image in tqdm.tqdm(image_list):
        seginfo, debug_image, _ = detic_wrapper.infer(image)
        result_dict['image'].append(image)
        result_dict['seginfo'].append(seginfo)
        result_dict['debug_image'].append(debug_image)

    with open(output_file_path, 'wb') as f:
        pickle.dump(result_dict, f)
