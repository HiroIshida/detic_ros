#!/usr/bin/env python3
import argparse
import dataclasses
import glob
import numpy as np
import os
import tempfile
import time
import warnings
import cv2
import tqdm
import sys

import torch
from detectron2.config import get_cfg
from detectron2.data.detection_utils import read_image
from detectron2.utils.logger import setup_logger

import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from detic_ros.msg import SegmentationInfo

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

class DeticRosNode:

    @dataclasses.dataclass
    class DummyArgs: 
        # Because VisualizationDemo class requires args
        vocabulary: str
        @classmethod
        def from_rosparam(cls): 
            vocabulary = rospy.get_param('~vocabulary', 'lvis')
            assert vocabulary in ['lvis', 'openimages', 'objects365', 'coco', 'custom']
            return cls(vocabulary)

    def __init__(self, cfg, dummy_args):
        self.predictor = VisualizationDemo(cfg, dummy_args)
        input_image = rospy.get_param('~input_image', '/kinect_head/rgb/half/image_rect_color')

        rospy.Subscriber(input_image, Image, self.callback)
        self.pub_debug_image = rospy.Publisher('debug_detic_image', Image, queue_size=10)
        self.pub_segmentation_image = rospy.Publisher('segmentation_image', Image, queue_size=10)
        self.pub_info = rospy.Publisher('segmentation_info', SegmentationInfo, queue_size=10)
        self.friendly_seg_value = rospy.get_param('~friendly_seg_value', False)


    @classmethod
    def from_rosparam(cls):
        cfg = cfg_from_rosparam()
        dummy_args = cls.DummyArgs.from_rosparam()
        return cls(cfg, dummy_args)


    def callback(self, msg):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        ts = time.time()
        predictions, visualized_output = self.predictor.run_on_image(img)
        instances = predictions['instances'].to(torch.device("cpu"))
        rospy.loginfo('elapsed time to inference {}'.format(time.time() - ts))

        # Create debug image
        msg_out = bridge.cv2_to_imgmsg(visualized_output.get_image(), encoding="passthrough")
        self.pub_debug_image.publish(msg_out)

        # Create Image containing segmentation info
        seg_img = Image(height=img.shape[0], width=img.shape[1])
        seg_img.encoding = '8UC1' # TODO(HiroIshida) are 256 classes enough??
        seg_img.is_bigendian = 0
        seg_img.step = seg_img.width * 1
        data = np.zeros((seg_img.height, seg_img.width)).astype(np.uint8)

        assert len(instances.pred_masks) - 1
        friendly_scaling = 256//len(instances.pred_masks + 1) if self.friendly_seg_value else 1
        for i, mask in enumerate(instances.pred_masks):
            # lable 0 is reserved for background label, so starting from 1
            data[mask] = (i + 1) * friendly_scaling
        seg_img.data = data.flatten().tolist()
        self.pub_segmentation_image.publish(seg_img)

        # Create segmentation info message
        class_names = self.predictor.metadata.get("thing_classes", None)
        class_indexes = instances.pred_classes.tolist()
        class_names_detected = ['background'] + [class_names[i] for i in class_indexes]
        seginfo = SegmentationInfo()
        seginfo.detected_classes = class_names_detected
        seginfo.scores = instances.scores
        self.pub_info.publish(seginfo)


if __name__=='__main__':
    rospy.init_node('detic_node', anonymous=True)
    node = DeticRosNode.from_rosparam()
    rospy.spin()
