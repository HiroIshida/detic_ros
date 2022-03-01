#!/usr/bin/env python3

import unittest
import rostest, rospy

class TestMyTest(unittest.TestCase):
    def setUp(self):
        pass

    def test_mytest2(self):
        pass

if __name__ == '__main__':
    rospy.init_node('test_sample')
    rostest.rosrun('mypkg',
                   'test_sample',
                   TestMyTest)
