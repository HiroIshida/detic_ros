#!/usr/bin/env python3
import copy
import numpy as np
import os
import sys
import torch
from typing import Optional
from detectron2.config import get_cfg

import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from detic_ros.msg import SegmentationInfo

# Dirty but no way, because CenterNet2 is not package oriented
sys.path.insert(0, os.path.join(sys.path[0], 'third_party/CenterNet2/projects/CenterNet2/'))
from centernet.config import add_centernet_config
import detic
from detic.config import add_detic_config
from detic.predictor import VisualizationDemo
from node_config import NodeConfig


def cfg_from_nodeconfig(node_config: NodeConfig):
    cfg = get_cfg()
    add_centernet_config(cfg)
    add_detic_config(cfg)
    cfg.merge_from_file(node_config.detic_config_path)
    cfg.MODEL.RETINANET.SCORE_THRESH_TEST = node_config.confidence_threshold
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = node_config.confidence_threshold
    cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = node_config.confidence_threshold
    cfg.merge_from_list(['MODEL.WEIGHTS', node_config.model_weights_path])

    # Similar to https://github.com/facebookresearch/Detic/demo.py
    cfg.MODEL.ROI_BOX_HEAD.ZEROSHOT_WEIGHT_PATH = 'rand' # load later
    cfg.MODEL.ROI_HEADS.ONE_CLASS_PER_PROPOSAL = True

    # Maybe should edit detic_configs/Base-C2_L_R5021k_640b64_4x.yaml
    pack_path = rospkg.RosPack().get_path('detic_ros')
    cfg.MODEL.ROI_BOX_HEAD.CAT_FREQ_PATH = os.path.join(
            pack_path, 'datasets/metadata/lvis_v1_train_cat_info.json')

    cfg.freeze()
    return cfg


class DeticRosNode:

    class DummyArgs: 
        vocabulary: str
        def __init__(self, vocabulary):
            assert vocabulary in ['lvis', 'openimages', 'objects365', 'coco', 'custom']
            self.vocabulary = vocabulary

    def __init__(self, node_config: Optional[NodeConfig]=None):
        if node_config is None:
            node_config = NodeConfig.from_rosparam()
        cfg = cfg_from_nodeconfig(node_config)
        dummy_args = self.DummyArgs(node_config.voabulary)
        self.predictor = VisualizationDemo(cfg, dummy_args)

        self.node_config = node_config
        self.sub = rospy.Subscriber('~input_image', Image, self.callback)
        self.pub_debug_image = rospy.Publisher('~debug_image', Image, queue_size=10)
        self.pub_segmentation_image = rospy.Publisher('~segmentation_image', Image, queue_size=10)
        if node_config.out_debug_segimage:
            self.pub_debug_segmentation_image = rospy.Publisher('~debug_segmentation_image', Image, queue_size=10)

        self.pub_info = rospy.Publisher('~segmentation_info', SegmentationInfo, queue_size=10)
        rospy.loginfo('initialized node')

    def callback(self, msg):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        time_start = rospy.Time.now()
        predictions, visualized_output = self.predictor.run_on_image(img)
        instances = predictions['instances'].to(torch.device("cpu"))

        if self.node_config.verbose:
            time_elapsed = (rospy.Time.now() - time_start).to_sec()
            rospy.loginfo('elapsed time to inference {}'.format(time_elapsed))
            rospy.loginfo('detected {} classes'.format(len(instances)))

        # Create debug image
        msg_out = bridge.cv2_to_imgmsg(visualized_output.get_image(), encoding="rgb8")
        self.pub_debug_image.publish(msg_out)

        # Create Image containing segmentation info
        seg_img = Image(height=img.shape[0], width=img.shape[1])
        seg_img.encoding = '8UC1' # TODO(HiroIshida) are 256 classes enough??
        seg_img.is_bigendian = 0
        seg_img.step = seg_img.width * 1
        data = np.zeros((seg_img.height, seg_img.width)).astype(np.uint8)

        assert len(instances.pred_masks) - 1
        for i, mask in enumerate(instances.pred_masks):
            # lable 0 is reserved for background label, so starting from 1
            data[mask] = (i + 1)
        assert data.shape == (seg_img.height, seg_img.width)
        seg_img.data = data.flatten().astype(np.uint8).tolist()
        self.pub_segmentation_image.publish(seg_img)

        if self.node_config.out_debug_segimage:
            debug_data = copy.deepcopy(data)
            human_friendly_scaling = 256//(len(instances.pred_masks) + 1)
            debug_data = debug_data * human_friendly_scaling
            debug_seg_img = copy.deepcopy(seg_img)
            debug_seg_img.data = debug_data.flatten().astype(np.uint8).tolist()
            self.pub_debug_segmentation_image.publish(debug_seg_img) 

        # Create segmentation info message
        class_names = self.predictor.metadata.get("thing_classes", None)
        class_indexes = instances.pred_classes.tolist()
        class_names_detected = ['background'] + [class_names[i] for i in class_indexes]
        seginfo = SegmentationInfo()
        seginfo.detected_classes = class_names_detected
        seginfo.scores = instances.scores
        self.pub_info.publish(seginfo)


def adhoc_hack_metadata_path():
    # because original BUILDIN_CLASSIFIER is somehow posi-dep
    rospack = rospkg.RosPack()
    pack_path = rospack.get_path('detic_ros')
    path_dict = detic.predictor.BUILDIN_CLASSIFIER
    for key in  path_dict.keys():
        path_dict[key] = os.path.join(pack_path, path_dict[key])


if __name__=='__main__':
    adhoc_hack_metadata_path()

    rospy.init_node('detic_node', anonymous=True)
    node = DeticRosNode()
    rospy.spin()
