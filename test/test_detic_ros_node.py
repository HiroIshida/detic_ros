#!/usr/bin/env python3
from logging import debug
import time

import unittest
import rostest, rospy
from sensor_msgs.msg import Image
from detic_ros.msg import SegmentationInfo


class TestNode(unittest.TestCase):
    def setUp(self):
        pass

    def test_mytest2(self):
        cb_data = {
                'msg1': None, 
                'msg2': None, 
                'msg3': None}
        def all_subscribed() -> bool:
            bool_list = [v is not None for v in cb_data.values()]
            return all(bool_list)

        def cb_debug_image(msg): cb_data['msg1'] = msg
        def cb_segimage(msg): cb_data['msg2'] = msg
        def cb_info(msg): cb_data['msg3'] = msg

        rospy.Subscriber('/docker/detic_segmentor/debug_image', Image, cb_debug_image)
        rospy.Subscriber('/docker/detic_segmentor/debug_segmentation_image', Image, cb_segimage)
        rospy.Subscriber('/docker/detic_segmentor/segmentation_info', SegmentationInfo, cb_info)

        time_out = 40
        for _ in range(time_out):
            time.sleep(1.0)
            if all_subscribed():
                return
        assert False


if __name__ == '__main__':
    rospy.init_node('test_sample')
    rostest.rosrun('detic_ros',
                   'test_node',
                   TestNode)
