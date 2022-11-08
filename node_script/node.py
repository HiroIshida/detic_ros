#!/usr/bin/env python3
from typing import Optional

import rospy
from rospy import Subscriber, Publisher
from sensor_msgs.msg import Image
from detic_ros.msg import SegmentationInfo
from detic_ros.srv import DeticSeg, DeticSegRequest, DeticSegResponse
from jsk_recognition_msgs.msg import LabelArray, VectorArray

from node_config import NodeConfig
from wrapper import DeticWrapper


class DeticRosNode:
    detic_wrapper: DeticWrapper
    sub: Subscriber
    pub_debug_image: Publisher
    pub_debug_segmentation_image: Publisher
    pub_info: Publisher
    pub_segimg: Publisher
    pub_labels: Publisher
    pub_score: Publisher

    def __init__(self, node_config: Optional[NodeConfig]=None):
        if node_config is None:
            node_config = NodeConfig.from_rosparam()

        self.detic_wrapper = DeticWrapper(node_config)
        self.srv_handler = rospy.Service('~segment_image', DeticSeg, self.callback_srv)

        if node_config.enable_pubsub:
            # As for large buff_size please see:
            # https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/?answer=220505?answer=220505#post-id-22050://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/?answer=220505?answer=220505#post-id-220505
            self.sub = rospy.Subscriber('~input_image', Image, self.callback_image, queue_size=1, buff_size=2**24)
            if node_config.use_jsk_msgs:
                self.pub_segimg = rospy.Publisher('~segmentation', Image, queue_size=1)
                self.pub_labels = rospy.Publisher('~detected_classes', LabelArray, queue_size=1)
                self.pub_score = rospy.Publisher('~score', VectorArray, queue_size=1)
            else:
                self.pub_info = rospy.Publisher('~segmentation_info', SegmentationInfo,
                                                queue_size=1)

            if node_config.out_debug_img:
                self.pub_debug_image = rospy.Publisher('~debug_image', Image, queue_size=1)
            else:
                self.pub_debug_image = None
            if node_config.out_debug_segimg:
                self.pub_debug_segmentation_image = rospy.Publisher('~debug_segmentation_image',
                                                                    Image, queue_size=10)
            else:
                self.pub_debug_segmentation_image = None

        rospy.loginfo('initialized node')

    def callback_image(self, msg: Image):
        # Inference
        seg_img, labels, scores, vis_img = self.detic_wrapper.inference_step(msg)

        # Publish main topics
        if self.detic_wrapper.node_config.use_jsk_msgs:
            self.pub_segimg.publish(seg_img)
            self.pub_labels.publish(self.detic_wrapper.get_label_array(labels))
            self.pub_score.publish(self.detic_wrapper.get_score_array(scores))
        else:
            seg_info = self.detic_wrapper.get_segmentation_info(seg_img, labels, scores)
            self.pub_info.publish(seg_info)

        # Publish optional topics
        if self.pub_debug_image is not None and vis_img is not None:
            debug_img = self.detic_wrapper.get_debug_img(vis_img)
            self.pub_debug_image.publish(debug_img)
        if self.pub_debug_segmentation_image is not None:
            debug_seg_img = self.detic_wrapper.get_debug_segimg()
            self.pub_debug_segmentation_image.publish(debug_seg_img)

        # Print debug info
        if self.detic_wrapper.node_config.verbose:
            time_elapsed_total = (rospy.Time.now() - msg.header.stamp).to_sec()
            rospy.loginfo('total elapsed time in callback {}'.format(time_elapsed_total))

    def callback_srv(self, req: DeticSegRequest) -> DeticSegResponse:
        msg = req.image
        seginfo, debug_img, _ = self.detic_wrapper.infer(msg)

        resp = DeticSegResponse()
        resp.seg_info = seginfo
        if debug_img is not None:
            resp.debug_image = debug_img
        return resp


if __name__=='__main__':
    rospy.init_node('detic_node', anonymous=True)
    node = DeticRosNode()
    rospy.spin()
