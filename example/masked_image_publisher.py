#!/usr/bin/env python3
import copy

import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from detic_ros.msg import SegmentationInfo


class SampleNode:

    def __init__(self, mask_class_name='bottle'):
        sub_image = message_filters.Subscriber(rospy.get_param('~in_image'), Image)
        sub_info = message_filters.Subscriber(rospy.get_param('~seginfo'), SegmentationInfo)
        sub_list = [sub_image, sub_info]
        ts = message_filters.ApproximateTimeSynchronizer(sub_list, 100, 10.0)
        ts.registerCallback(self.callback)

        self.pub = rospy.Publisher(rospy.get_param('~out_image'), Image,
                                   queue_size=1)
        self.class_name = mask_class_name

    def callback(self, msg_image, msg_info: SegmentationInfo):
        rospy.loginfo('rec messages')

        # simply republish if class name is 'background'
        if (self.class_name == 'background'):
            self.pub.publish(msg_image)
            return

        # find label number corresponding to desired object class name
        try:
            label_index = msg_info.detected_classes.index(self.class_name)
            confidence_score = msg_info.scores[label_index]
            rospy.loginfo('specified object class {} is detected with score {}'.format(
                self.class_name, confidence_score))
        except ValueError:
            rospy.logdebug('class not found: {}'.format(self.class_name))
            return

        seg_img = msg_info.segmentation
        assert msg_image.width == seg_img.width
        assert msg_image.height == seg_img.height
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg_image, desired_encoding='passthrough')

        # Add 1 to label_index to account for the background
        mask_indexes = np.where(img == label_index + 1)

        masked_img = copy.deepcopy(img)
        masked_img[mask_indexes] = 0  # filled by black

        msg_out = bridge.cv2_to_imgmsg(masked_img, encoding="rgb8")
        self.pub.publish(msg_out)


if __name__ == '__main__':
    import sys
    argv = [x for x in sys.argv if not x.startswith('_')]  # remove roslaunch args
    if len(argv) == 1:
        class_name = 'background'
    elif len(argv) == 2:
        class_name = argv[1]
    else:
        print('Usage: masked_image_publisher.py [class_name]')
        sys.exit(1)

    rospy.init_node('mask_image_publisher', anonymous=True)
    SampleNode(mask_class_name=class_name)
    rospy.spin()
    pass
