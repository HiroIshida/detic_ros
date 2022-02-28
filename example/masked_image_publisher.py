import copy
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image
from detic_ros.msg import SegmentationInfo
from cv_bridge import CvBridge

def segmentation_image_to_nparray(msg_seg):
    buf = np.ndarray(shape=(1, int(len(msg_seg.data))),
                      dtype=np.uint8, buffer=msg_seg.data)
    return buf.reshape(msg_seg.height, msg_seg.width)


class SampleNode:

    def __init__(self, mask_class_name='bottle'):
        image_topic_name = '/kinect_head/rgb/half/image_rect_color'
        sub_image = message_filters.Subscriber(image_topic_name, Image)
        sub_segmentation = message_filters.Subscriber('/docker/detic_segmentor/segmentation_image', Image)
        sub_info = message_filters.Subscriber('/docker/detic_segmentor/segmentation_info', SegmentationInfo)
        sub_list = [sub_image, sub_segmentation, sub_info]
        ts = message_filters.ApproximateTimeSynchronizer(sub_list, 100, 10.0)
        ts.registerCallback(self.callback)

        self.pub = rospy.Publisher('sample_masked_image', Image)
        self.class_name = mask_class_name

    def callback(self, msg_image, msg_segmentation, msg_info: SegmentationInfo):
        rospy.loginfo('rec messages')

        # find label number corresponding to desired object class name
        try:
            label_index = msg_info.detected_classes.index(self.class_name)
            confidence_score = msg_info.scores[label_index]
            rospy.loginfo('specified object class {} is detected with score {}'.format(
                self.class_name, confidence_score))
        except ValueError:
            return

        assert msg_image.width == msg_segmentation.width
        assert msg_image.height == msg_segmentation.height
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg_image, desired_encoding='passthrough')

        seg_matrix = segmentation_image_to_nparray(msg_segmentation)
        mask_indexes = np.where(seg_matrix==label_index)

        masked_img = copy.deepcopy(img)
        masked_img[mask_indexes] = np.zeros(3, dtype=np.uint8) # filled by black

        msg_out = bridge.cv2_to_imgmsg(masked_img, encoding="rgb8")
        self.pub.publish(msg_out)


if __name__=='__main__':
    rospy.init_node('mask_image_publisher', anonymous=True)
    SampleNode(mask_class_name='background')
    rospy.spin()


