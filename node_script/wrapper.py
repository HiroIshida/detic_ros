import os
import copy
from typing import Optional, Tuple

import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from detic_ros.msg import SegmentationInfo
import torch
import numpy as np

import detic
from detic.predictor import VisualizationDemo
from node_config import NodeConfig


class DeticWrapper:
    predictor: VisualizationDemo
    node_config: NodeConfig

    class DummyArgs: 
        vocabulary: str
        def __init__(self, vocabulary):
            assert vocabulary in ['lvis', 'openimages', 'objects365', 'coco', 'custom']
            self.vocabulary = vocabulary

    def __init__(self, node_config: NodeConfig):
        self._adhoc_hack_metadata_path()
        detectron_cfg = node_config.to_detectron_config()
        dummy_args = self.DummyArgs(node_config.vocabulary)

        self.predictor = VisualizationDemo(detectron_cfg, dummy_args)
        self.node_config = node_config

    @staticmethod
    def _adhoc_hack_metadata_path():
        # because original BUILDIN_CLASSIFIER is somehow posi-dep
        rospack = rospkg.RosPack()
        pack_path = rospack.get_path('detic_ros')
        path_dict = detic.predictor.BUILDIN_CLASSIFIER
        for key in  path_dict.keys():
            path_dict[key] = os.path.join(pack_path, path_dict[key])


    def infer(self, msg: Image) -> Tuple[SegmentationInfo, Optional[Image], Optional[Image]]:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        if self.node_config.verbose:
            time_start = rospy.Time.now()

        predictions, visualized_output = self.predictor.run_on_image(img)
        instances = predictions['instances'].to(torch.device("cpu"))

        if self.node_config.verbose:
            time_elapsed = (rospy.Time.now() - time_start).to_sec()
            rospy.loginfo('elapsed time to inference {}'.format(time_elapsed))
            rospy.loginfo('detected {} classes'.format(len(instances)))

        # Create debug image
        if self.node_config.out_debug_img:
            debug_img = bridge.cv2_to_imgmsg(visualized_output.get_image(), encoding="rgb8")
            debug_img.header = msg.header
        else:
            debug_img = None

        # Create Image containing segmentation info
        seg_img = Image(height=img.shape[0], width=img.shape[1])
        seg_img.header = msg.header
        seg_img.encoding = '8UC1' # TODO(HiroIshida) are 256 classes enough??
        seg_img.is_bigendian = 0
        seg_img.step = seg_img.width * 1
        data = np.zeros((seg_img.height, seg_img.width)).astype(np.uint8)

        # largest to smallest order to reduce occlusion.
        sorted_index = np.argsort([-mask.sum() for mask in instances.pred_masks])
        for i in sorted_index:
            mask = instances.pred_masks[i]
            # lable 0 is reserved for background label, so starting from 1
            data[mask] = (i + 1)
        assert data.shape == (seg_img.height, seg_img.width)
        seg_img.data = data.flatten().astype(np.uint8).tolist()
        assert set(seg_img.data) == set(list(range(len(instances.pred_masks)+1)))

        if self.node_config.out_debug_segimage:
            debug_data = copy.deepcopy(data)
            human_friendly_scaling = 256//(len(instances.pred_masks) + 1)
            debug_data = debug_data * human_friendly_scaling
            debug_seg_img = copy.deepcopy(seg_img)
            debug_seg_img.data = debug_data.flatten().astype(np.uint8).tolist()
        else:
            debug_seg_img = None

        # Create segmentation info message
        class_names = self.predictor.metadata.get("thing_classes", None)
        class_indexes = instances.pred_classes.tolist()
        class_names_detected = ['background'] + [class_names[i] for i in class_indexes]
        seginfo = SegmentationInfo()
        seginfo.detected_classes = class_names_detected
        seginfo.scores = [1.0] + instances.scores.tolist() # confidence with 1.0 about background detection
        seginfo.header = msg.header
        seginfo.segmentation = seg_img

        if self.node_config.verbose:
            time_elapsed_total = (rospy.Time.now() - time_start).to_sec()
            rospy.loginfo('total elapsed time in callback {}'.format(time_elapsed_total))

            total_publication_delay = (rospy.Time.now() - msg.header.stamp).to_sec()
            responsibility = (time_elapsed_total / total_publication_delay) * 100.0
            rospy.loginfo('total delay {} sec (this cb {} %)'.format(total_publication_delay, responsibility))
        return seginfo, debug_img, debug_seg_img
