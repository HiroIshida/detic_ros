#!/usr/bin/env python3
import os
import unittest

import pickle
import rostest, rospy
import rospkg


class TestNode(unittest.TestCase):
    def setUp(self):
        pass

    def test_pkl_dump(self):
        pkg_path = rospkg.RosPack().get_path('detic_ros')
        bag_path = os.path.join(pkg_path, 'test', 'data', 'desk.bag')
        ret = os.system('rosrun detic_ros batch_processor.py {} -n 1'.format(bag_path))
        assert ret == 0

        pkl_path = os.path.join(pkg_path, 'test', 'data', 'desk_segmented.pkl')
        with open(pkl_path, 'rb') as f:
            obj = pickle.load(f)

    def test_rosbag_dump(self):
        pkg_path = rospkg.RosPack().get_path('detic_ros')
        bag_path = os.path.join(pkg_path, 'test', 'data', 'desk.bag')
        ret = os.system('rosrun detic_ros batch_processor.py {} -n 1 -format bag'.format(bag_path))
        assert ret == 0

        outbag_path = os.path.join(pkg_path, 'test', 'data', 'desk_segmented.bag')


if __name__ == '__main__':
    rospy.init_node('test_batch_processor')
    rostest.rosrun('detic_ros',
                   'test_node',
                   TestNode)
