#!/usr/bin/env python3
import os
import unittest

import pickle
import rostest, rospy
import rospkg


class TestNode(unittest.TestCase):
    def setUp(self):
        pass

    def test_mytest(self):
        pkg_path = rospkg.RosPack().get_path('detic_ros')
        bag_path = os.path.join(pkg_path, 'test', 'data', 'desk.bag -out /tmp/a.pkl')
        os.system('rosrun detic_ros batch_processor.py {} -n 1'.format(bag_path))

        with open('/tmp/a.pkl', 'rb') as f:
            result_dict = pickle.load(f)

        assert len(result_dict['image']) == 1
        assert len(result_dict['seginfo']) == 1
        assert len(result_dict['debug_image']) == 1


if __name__ == '__main__':
    rospy.init_node('test_batch_processor')
    rostest.rosrun('detic_ros',
                   'test_node',
                   TestNode)
