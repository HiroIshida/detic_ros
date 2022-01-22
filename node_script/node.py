#!/usr/bin/env python3
import argparse
import dataclasses
import glob
import multiprocessing as mp
import numpy as np
import os
import tempfile
import time
import warnings
import cv2
import tqdm
import sys

from detectron2.config import get_cfg
from detectron2.data.detection_utils import read_image
from detectron2.utils.logger import setup_logger

import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.join(sys.path[0], 'third_party/CenterNet2/projects/CenterNet2/'))

from centernet.config import add_centernet_config
from detic.config import add_detic_config
from detic.predictor import VisualizationDemo


def cfg_from_rosparam():
    """ 
    setup detctron2 config
    """

    rospack = rospkg.RosPack()
    pack_path = rospack.get_path('detic_ros')
    default_detic_config_path = os.path.join(
        pack_path, 'detic_configs', 
        'Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml')

    default_model_weights_path = os.path.join(
        pack_path, 'models',
        'Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth')


    detic_config_path = rospy.get_param('~detic_config_path', default_detic_config_path)
    confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
    model_weights_path = rospy.get_param('~model_weights_path', default_model_weights_path)


    cfg = get_cfg()
    add_centernet_config(cfg)
    add_detic_config(cfg)
    cfg.merge_from_file(detic_config_path)
    cfg.MODEL.RETINANET.SCORE_THRESH_TEST = confidence_threshold
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = confidence_threshold
    cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = confidence_threshold
    cfg.merge_from_list(['MODEL.WEIGHTS', model_weights_path])

    # Similar to https://github.com/facebookresearch/Detic/demo.py
    cfg.MODEL.ROI_BOX_HEAD.ZEROSHOT_WEIGHT_PATH = 'rand' # load later
    cfg.MODEL.ROI_HEADS.ONE_CLASS_PER_PROPOSAL = True

    # Maybe should edit detic_configs/Base-C2_L_R5021k_640b64_4x.yaml
    cfg.MODEL.ROI_BOX_HEAD.CAT_FREQ_PATH = os.path.join(
            pack_path, 'datasets/metadata/lvis_v1_train_cat_info.json')

    cfg.freeze()
    return cfg

if __name__=='__main__':
    cfg = cfg_from_rosparam()


    @dataclasses.dataclass
    class DummyArgs: 
        # Because VisualizationDemo class requires args
        vocabulary: str
        @classmethod
        def from_rosparam(cls): 
            vocabulary = rospy.get_param('~vocabulary', 'lvis')
            assert vocabulary in ['lvis', 'openimages', 'objects365', 'coco', 'custom']
            return cls(vocabulary)


    mp.set_start_method("spawn", force=True)
    # TODO(HiroIshida) add logger ??
    demo = VisualizationDemo(cfg, DummyArgs.from_rosparam())


    rospy.init_node('detic_node', anonymous=True)
    input_image = rospy.get_param('~input_image', '/kinect_head/rgb/image_color')
    pub = rospy.Publisher('debug_detic_image', Image, queue_size=10)
    def callback(msg):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        predictions, visualized_output = demo.run_on_image(img)
        msg_out = bridge.cv2_to_imgmsg(visualized_output.get_image(), encoding="passthrough")
        pub.publish(msg_out)
        rospy.loginfo('published image')

    rospy.Subscriber(input_image, Image, callback)
    rospy.spin()
