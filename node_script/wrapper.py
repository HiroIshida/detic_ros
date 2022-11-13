import os
from dataclasses import dataclass
from typing import List, Optional

import detic
import numpy as np
import rospkg
import rospy
import torch
from cv_bridge import CvBridge
from detectron2.utils.visualizer import VisImage
from detic.predictor import VisualizationDemo
from jsk_recognition_msgs.msg import Label, LabelArray, VectorArray
from node_config import NodeConfig
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from detic_ros.msg import SegmentationInfo

_cv_bridge = CvBridge()


@dataclass(frozen=True)
class InferenceRawResult:
    segmentation_raw_image: np.ndarray
    class_indices: List[int]
    scores: List[float]
    visualization: Optional[VisImage]
    header: Header
    detected_class_names: List[str]

    def get_ros_segmentaion_image(self) -> Image:
        seg_img = _cv_bridge.cv2_to_imgmsg(self.segmentation_raw_image, encoding="32SC1")
        seg_img.header = self.header
        return seg_img

    def get_ros_debug_image(self) -> Image:
        message = "you didn't configure the wrapper so that it computes the debug images"
        assert self.visualization is not None, message
        debug_img = _cv_bridge.cv2_to_imgmsg(
            self.visualization.get_image(), encoding="rgb8")
        debug_img.header = self.header
        return debug_img

    def get_ros_debug_segmentation_img(self) -> Image:
        human_friendly_scaling = 255 // self.segmentation_raw_image.max()
        new_data = (self.segmentation_raw_image * human_friendly_scaling).astype(np.uint8)
        debug_seg_img = _cv_bridge.cv2_to_imgmsg(new_data, encoding="mono8")
        debug_seg_img.header = self.header
        return debug_seg_img

    def get_label_array(self) -> LabelArray:
        labels = [Label(id=i + 1, name=name)
                  for i, name
                  in zip(self.class_indices, self.detected_class_names)]
        lab_arr = LabelArray(header=self.header, labels=labels)
        return lab_arr

    def get_score_array(self) -> VectorArray:
        vec_arr = VectorArray(header=self.header, vector_dim=len(self.scores), data=self.scores)
        return vec_arr

    def get_segmentation_info(self) -> SegmentationInfo:
        seg_img = self.get_ros_segmentaion_image()
        seg_info = SegmentationInfo(detected_classes=self.detected_class_names,
                                    scores=self.scores,
                                    segmentation=seg_img,
                                    header=self.header)
        return seg_info


class DeticWrapper:
    predictor: VisualizationDemo
    node_config: NodeConfig
    class_names: List[str]

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
        self.class_names = self.predictor.metadata.get("thing_classes", None)

    @staticmethod
    def _adhoc_hack_metadata_path():
        # because original BUILDIN_CLASSIFIER is somehow position dependent
        rospack = rospkg.RosPack()
        pack_path = rospack.get_path('detic_ros')
        path_dict = detic.predictor.BUILDIN_CLASSIFIER
        for key in path_dict.keys():
            path_dict[key] = os.path.join(pack_path, path_dict[key])

    def infer(self, msg: Image) -> InferenceRawResult:
        # Segmentation image, detected classes, detection scores, visualization image
        img = _cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

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

        # Initialize segmentation data
        data = np.zeros((img.shape[0], img.shape[1]), dtype=np.int32)

        # largest to smallest order to reduce occlusion.
        sorted_index = np.argsort([-mask.sum() for mask in instances.pred_masks])
        for i in sorted_index:
            mask = instances.pred_masks[i]
            # label 0 is reserved for background label, so starting from 1
            data[mask] = (i + 1)

        # Get class and score arrays
        class_indices = instances.pred_classes.tolist()
        detected_classes_names = [self.class_names[i] for i in class_indices]
        scores = instances.scores.tolist()
        result = InferenceRawResult(
            data,
            class_indices,
            scores,
            visualized_output,
            msg.header,
            detected_classes_names)
        return result
