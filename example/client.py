#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Image
from detic_ros.srv import DeticSeg

def numpy_to_imgmsg(data: np.ndarray) -> Image:
    # see: cv_bridge/core.py
    img_msg = Image()
    img_msg.height = data.shape[0]
    img_msg.width = data.shape[1]
    img_msg.encoding = 'bgr8'
    img_msg.data = data.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height
    img_msg.is_bigendian = True
    return img_msg

rospy.init_node('example_client')
rospy.wait_for_service('/docker/detic_segmentor/segment_image')
arr = np.random.randint(0, high=255, size=(100, 100, 3), dtype=np.uint8)
image_msg = numpy_to_imgmsg(arr)

f = rospy.ServiceProxy('/docker/detic_segmentor/segment_image', DeticSeg)
resp = f(image_msg)
seg_info = resp.seg_info
