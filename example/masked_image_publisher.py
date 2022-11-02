#!/usr/bin/env python3
import copy
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image
from detic_ros.msg import SegmentationInfo
from cv_bridge import CvBridge


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

        # find label number corresponding to desired object class name
        try:
            label_index = msg_info.detected_classes.index(self.class_name)
            confidence_score = msg_info.scores[label_index]
            rospy.loginfo('specified object class {} is detected with score {}'.format(
                self.class_name, confidence_score))
        except ValueError:
            return

        seg_img = msg_info.segmentation
        assert msg_image.width == seg_img.width
        assert msg_image.height == seg_img.height
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg_image, desired_encoding='passthrough')

        mask_indexes = np.where(img==label_index)

        masked_img = copy.deepcopy(img)
        masked_img[mask_indexes] = 0  # filled by black

        msg_out = bridge.cv2_to_imgmsg(masked_img, encoding="rgb8")
        self.pub.publish(msg_out)


if __name__=='__main__':
    rospy.init_node('mask_image_publisher', anonymous=True)
    SampleNode(mask_class_name='background')
    rospy.spin()
    pass


