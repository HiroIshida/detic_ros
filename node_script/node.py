import argparse
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

sys.path.insert(0, os.path.join(sys.path[0], 'third_party/CenterNet2/projects/CenterNet2/'))

from centernet.config import add_centernet_config
from detic.config import add_detic_config
from detic.predictor import VisualizationDemo


def cfg_from_rosparam():
    """ 
    setup detctron2 config
    Similar to https://github.com/facebookresearch/Detic/demo.py
    """

    rospack = rospkg.RosPack()
    default_detic_config_path = os.path.join(
        rospack.get_path('detic_ros'), 'detic_configs', 
        'Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml')

    default_model_weights_path = os.path.join(
        rospack.get_path('detic_ros'), 'models',
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
    return cfg

if __name__=='__main__':
    cfg = cfg_from_rosparam()
