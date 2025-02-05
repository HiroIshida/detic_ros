#!/usr/bin/env python3
import argparse

import matplotlib.pyplot as plt
import rospy
from cv_bridge import CvBridge
from detic_ros.srv import DeticSeg

import cv2

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-image', type=str, help='input image path')
    args = parser.parse_args()
    image = cv2.imread(args.image)
    assert image is not None

    rospy.init_node('example_client')
    rospy.wait_for_service('/docker/detic_segmentor/segment_image')
    image_msg = CvBridge().cv2_to_imgmsg(image, encoding='passthrough')

    f = rospy.ServiceProxy('/docker/detic_segmentor/segment_image', DeticSeg)
    resp = f(image_msg)
    seg_info = resp.seg_info
    if resp.debug_image is not None:
        image = CvBridge().imgmsg_to_cv2(resp.debug_image)
        plt.imshow(image)
        plt.show()
