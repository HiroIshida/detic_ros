import os
import copy
from typing import Optional, Tuple, List

import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from detic_ros.msg import SegmentationInfo


import torch
import numpy as np

import detic
from detic.predictor import VisualizationDemo
from jsk_recognition_msgs.msg import Label, LabelArray, VectorArray
from detectron2.utils.visualizer import VisImage
from node_config import NodeConfig


class DeticWrapper:
    predictor: VisualizationDemo
    node_config: NodeConfig

    class DummyArgs: 
        vocabulary: str
        def __init__(self, vocabulary, custom_vocabulary):
            assert vocabulary in ['lvis', 'openimages', 'objects365', 'coco', 'custom']
            self.vocabulary = vocabulary
            self.custom_vocabulary = custom_vocabulary

    def __init__(self, node_config: NodeConfig):
        self._adhoc_hack_metadata_path()
        detectron_cfg = node_config.to_detectron_config()
        dummy_args = self.DummyArgs(node_config.vocabulary, node_config.custom_vocabulary)

        self.predictor = VisualizationDemo(detectron_cfg, dummy_args)
        self.node_config = node_config
        self.bridge = CvBridge()
        self.class_names = self.predictor.metadata.get("thing_classes", None)
        self.seg_img = None

    @staticmethod
    def _adhoc_hack_metadata_path():
        # because original BUILDIN_CLASSIFIER is somehow posi-dep
        rospack = rospkg.RosPack()
        pack_path = rospack.get_path('detic_ros')
        path_dict = detic.predictor.BUILDIN_CLASSIFIER
        for key in  path_dict.keys():
            path_dict[key] = os.path.join(pack_path, path_dict[key])

    def infer(self, msg: Image) -> Tuple[Image, List[str], List[float], Optional[VisImage]]:
        # Segmentation image, detected classes, detection scores, visualization image
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        if self.node_config.verbose:
            time_start = rospy.Time.now()

        if self.node_config.out_debug_img:
            predictions, visualized_output = self.predictor.run_on_image(img)
        else:
            predictions = self.predictor.predictor(img)
            visualized_output = None
        instances = predictions['instances'].to(torch.device("cpu"))

        if self.node_config.verbose:
            time_elapsed = (rospy.Time.now() - time_start).to_sec()
            rospy.loginfo('elapsed time to inference {}'.format(time_elapsed))
            rospy.loginfo('detected {} classes'.format(len(instances)))

        # Create Image containing segmentation info
        if self.seg_img is None:
            self.seg_img = Image(height=img.shape[0], width=img.shape[1])
            self.seg_img.header = msg.header
            self.seg_img.encoding = '8UC1' # TODO(HiroIshida) are 256 classes enough??
            self.seg_img.is_bigendian = 0
            self.seg_img.step = self.seg_img.width * 1
        data = np.zeros((self.seg_img.height, self.seg_img.width), dtype=np.uint8)

        # largest to smallest order to reduce occlusion.
        sorted_index = np.argsort([-mask.sum() for mask in instances.pred_masks])
        for i in sorted_index:
            mask = instances.pred_masks[i]
            # lable 0 is reserved for background label, so starting from 1
            data[mask] = (i + 1)
        self.seg_img.data = data.flatten().astype(np.uint8).tolist()
        # assert set(self.seg_img.data) == set(list(range(len(instances.pred_masks)+1)))

        # Get class and score arrays
        class_indexes = instances.pred_classes.tolist()
        class_names_detected = ['background'] + [self.class_names[i] for i in class_indexes]
        scores = [1.0] + instances.scores.tolist() # confidence with 1.0 about background detection
        return self.seg_img, class_names_detected, scores, visualized_output

    def get_debug_img(self, visualized_output: VisImage) -> Image:
        # Call after infer
        debug_img = self.bridge.cv2_to_imgmsg(visualized_output.get_image(),
                                              encoding="rgb8")
        debug_img.header = self.seg_img.header
        return debug_img

    def get_debug_segimg(self) -> Image:
        # Call after infer
        debug_seg_img = copy.deepcopy(self.seg_img)
        human_friendly_scaling = 255 // debug_seg_img.data.max()
        debug_seg_img.data *= human_friendly_scaling
        return debug_seg_img

    def get_segmentation_info(self, detected_classes: List[str],
                              scores: List[float]) -> SegmentationInfo:
        seg_info = SegmentationInfo(detected_classes=detected_classes,
                                    scores=scores,
                                    segmentation=self.seg_img,
                                    header=self.seg_img.header)
        return seg_info

    def get_label_array(self, detected_classes: List[str]) -> LabelArray:
        labels = [Label(id=i, name=cls) for i,cls in enumerate(detected_classes)]
        lab_arr = LabelArray(header=self.seg_img.header,
                             labels=labels)
        return lab_arr

    def get_score_array(self, scores: List[float]) -> VectorArray:
        vec_arr = VectorArray(header=self.seg_img.header,
                              vector_dim=len(scores),
                              data=scores)
        return vec_arr
