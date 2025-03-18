#!/usr/bin/env python3
from typing import Optional

import rospy
import torch
from jsk_recognition_msgs.msg import LabelArray, VectorArray
from jsk_topic_tools.transport import ConnectionBasedTransport
from rospy import Publisher, Subscriber
from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

from detic_ros.msg import SegmentationInfo
from detic_ros.node_config import NodeConfig
from detic_ros.srv import (
    CustomVocabulary,
    CustomVocabularyRequest,
    CustomVocabularyResponse,
    DeticSeg,
    DeticSegRequest,
    DeticSegResponse,
)
from detic_ros.wrapper import DeticWrapper


class DeticRosNode(ConnectionBasedTransport):
    is_active: bool
    detic_wrapper: DeticWrapper
    sub: Subscriber
    # some debug image publisher
    pub_debug_image: Optional[Publisher]
    pub_debug_segmentation_image: Optional[Publisher]

    # used when you set use_jsk_msgs = True
    pub_segimg: Optional[Publisher]
    pub_labels: Optional[Publisher]
    pub_score: Optional[Publisher]

    _node_config: NodeConfig

    # otherwise, the following publisher will be used
    pub_info: Optional[Publisher]

    def __init__(self, node_config: Optional[NodeConfig] = None):
        super(DeticRosNode, self).__init__()

        if node_config is None:
            self._node_config = NodeConfig.from_rosparam()

        rospy.loginfo("node_config: {}".format(self._node_config))

        self.is_active = True
        self.detic_wrapper = DeticWrapper(self._node_config)
        self.srv_handler = rospy.Service('~segment_image', DeticSeg, self.callback_srv)
        self.vocab_srv_handler = rospy.Service('~custom_vocabulary', CustomVocabulary, self.custom_vocab_srv)
        self.inactivate_srv_handler = rospy.Service('~inactivate', Empty, self.inactivate_srv)
        self.activate_srv_handler = rospy.Service('~activate', Empty, self.activate_srv)
        self.default_vocab_srv_handler = rospy.Service('~default_vocabulary', Empty, self.default_vocab_srv)

        if self._node_config.num_torch_thread is not None:
            torch.set_num_threads(self._node_config.num_torch_thread)

        if not self._node_config.enable_pubsub: return
        if self._node_config.use_jsk_msgs:
            self.pub_segimg = self.advertise('~segmentation', Image, queue_size=1)
            self.pub_labels = self.advertise('~detected_classes', LabelArray, queue_size=1)
            self.pub_score = self.advertise('~score', VectorArray, queue_size=1)
        else:
            self.pub_info = self.advertise('~segmentation_info', SegmentationInfo,
                                           queue_size=1)

        if self._node_config.out_debug_img:
            self.pub_debug_image = self.advertise('~debug_image', Image, queue_size=1)
        else:
            self.pub_debug_image = None
        if self._node_config.out_debug_segimg:
            self.pub_debug_segmentation_image = self.advertise('~debug_segmentation_image',
                                                                    Image, queue_size=10)
        else:
            self.pub_debug_segmentation_image = None

    def subscribe(self):
        if self._node_config.enable_pubsub:
            # As for large buff_size please see:
            # https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/?answer=220505?answer=220505#post-id-22050://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/?answer=220505?answer=220505#post-id-220505
            self.sub = rospy.Subscriber('~input_image', Image, self.callback_image, queue_size=1, buff_size=2**24)


    def unsubscribe(self):
        self.sub.unregister()

    def callback_image(self, msg: Image):
        time_start = rospy.Time.now()
        if not self.is_active:
            rospy.sleep(0.1)  # to avoid potential busy loop
            return

        raw_result = self.detic_wrapper.infer(msg)

        # Publish main topics
        if self.detic_wrapper.node_config.use_jsk_msgs:
            # assertion for mypy
            assert self.pub_segimg is not None
            assert self.pub_labels is not None
            assert self.pub_score is not None
            seg_img = raw_result.get_ros_segmentaion_image()
            labels = raw_result.get_label_array()
            scores = raw_result.get_score_array()
            self.pub_segimg.publish(seg_img)
            self.pub_labels.publish(labels)
            self.pub_score.publish(scores)
        else:
            assert self.pub_info is not None
            seg_info = raw_result.get_segmentation_info()
            self.pub_info.publish(seg_info)

        # Publish optional topics

        if self.pub_debug_image is not None:
            debug_img = raw_result.get_ros_debug_image()
            self.pub_debug_image.publish(debug_img)

        if self.pub_debug_segmentation_image is not None:
            debug_seg_img = raw_result.get_ros_debug_segmentation_img()
            self.pub_debug_segmentation_image.publish(debug_seg_img)

        # Print debug info
        if self.detic_wrapper.node_config.verbose:
            time_elapsed_total = (rospy.Time.now() - time_start).to_sec()
            rospy.loginfo('total elapsed time in callback {}'.format(time_elapsed_total))

    def inactivate_srv(self, req: EmptyRequest) -> EmptyResponse:
        rospy.loginfo("Inactivate subscriber callback")
        self.is_active = False
        return EmptyResponse()

    def activate_srv(self, req: EmptyRequest) -> EmptyResponse:
        rospy.loginfo("Activate subscriber callback")
        self.is_active = True
        return EmptyResponse()

    def callback_srv(self, req: DeticSegRequest) -> DeticSegResponse:
        msg = req.image
        raw_result = self.detic_wrapper.infer(msg)
        seginfo = raw_result.get_segmentation_info()

        resp = DeticSegResponse()
        resp.seg_info = seginfo

        if raw_result.visualization is not None:
            debug_image = raw_result.get_ros_debug_segmentation_img()
            resp.debug_image = debug_image
        return resp

    def custom_vocab_srv(self, req: CustomVocabularyRequest) -> CustomVocabularyResponse:
        rospy.loginfo("Change vocabulary to {}".format(req.vocabulary))
        self.detic_wrapper.change_vocabulary(req.vocabulary)
        res = CustomVocabularyResponse()
        return res

    def default_vocab_srv(self, req: EmptyRequest) -> EmptyResponse:
        rospy.loginfo("Change to default vocabulary")
        self.detic_wrapper.set_default_vocabulary()
        res = EmptyResponse()
        return res


if __name__ == '__main__':
    rospy.init_node('detic_node', anonymous=True)
    node = DeticRosNode()
    rospy.spin()
