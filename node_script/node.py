import rospy
import rospkg

import argparse
import glob
import multiprocessing as mp
import numpy as np
import os
import tempfile
import time
import warnings
import cv2
import tqdm
import sys

from detectron2.config import get_cfg
from detectron2.data.detection_utils import read_image
from detectron2.utils.logger import setup_logger

sys.path.insert(0, 'third_party/CenterNet2/projects/CenterNet2/')
from centernet.config import add_centernet_config
from detic.config import add_detic_config

from detic.predictor import VisualizationDemo

rospack = rospkg.RosPack()
detic_config_path = os.path.join(
    rospack.get_path('detic_ros'),
    'models', 
    'Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth')
model_path = rospy.get_param('~model_path')
