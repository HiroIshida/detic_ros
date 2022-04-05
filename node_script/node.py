#!/usr/bin/env python3
from typing import Optional

import rospy
from rospy import Subscriber, Publisher
from sensor_msgs.msg import Image
from detic_ros.msg import SegmentationInfo
from detic_ros.srv import DeticSeg, DeticSegRequest, DeticSegResponse

from node_config import NodeConfig
from wrapper import DeticWrapper


class DeticRosNode:
    detic_wrapper: DeticWrapper
    sub: Subscriber
    pub_debug_image: Publisher
    pub_debug_segmentation_image: Publisher
    pub_info: Publisher

    def __init__(self, node_config: Optional[NodeConfig]=None):
        if node_config is None:
            node_config = NodeConfig.from_rosparam()

        self.detic_wrapper = DeticWrapper(node_config)

        self.srv_handler = rospy.Service('~segment_image', DeticSeg, self.callback_srv)

        if node_config.enable_pubsub:
            # As for large buff_size please see:
            # https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/?answer=220505?answer=220505#post-id-22050://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/?answer=220505?answer=220505#post-id-220505
            self.sub = rospy.Subscriber('~input_image', Image, self.callback_image, queue_size=1, buff_size=2**24)
            self.pub_debug_image = rospy.Publisher('~debug_image', Image, queue_size=1)
            self.pub_debug_segmentation_image = rospy.Publisher('~debug_segmentation_image', Image, queue_size=10)
            self.pub_info = rospy.Publisher('~segmentation_info', SegmentationInfo, queue_size=1)

        rospy.loginfo('initialized node')

    def callback_image(self, msg: Image):
        seginfo, debug_img, debug_seg_img = self.detic_wrapper.infer(msg)
        self.pub_info.publish(seginfo)
        if debug_img is not None:
            self.pub_debug_image.publish(debug_img)
        if debug_seg_img is not None:
            self.pub_debug_segmentation_image.publish(debug_seg_img)

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
