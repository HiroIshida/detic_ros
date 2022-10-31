import os
import sys
from dataclasses import dataclass
import rospy
import rospkg

import torch

# Dirty but no way, because CenterNet2 is not package oriented
sys.path.insert(0, os.path.join(sys.path[0], 'third_party/CenterNet2/'))

from detectron2.config import get_cfg
from centernet.config import add_centernet_config
from detic.config import add_detic_config

# model_name = 'Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size'
model_name = 'Detic_LCOCOI21k_CLIP_CXT21k_640b32_4x_ft4x_max-size'

@dataclass
class NodeConfig:
    enable_pubsub: bool
    out_debug_img: bool
    out_debug_segimg: bool
    verbose: bool
    vocabulary: str
    custom_vocabulary: str
    detic_config_path: str
    model_weights_path: str
    confidence_threshold: float
    device_name: str

    @classmethod
    def from_args(cls, 
            enable_pubsub: bool = True,
            out_debug_img: bool = True,
            out_debug_segimg: bool = True,
            verbose: bool = False,
            confidence_threshold: float = 0.5,
            device_name: str = 'auto',
            vocabulary: str = 'lvis',
            custom_vocabulary: str = '',
            ):

        if device_name == 'auto':
            device_name = 'cuda' if torch.cuda.is_available() else 'cpu'

        assert device_name in ['cpu', 'cuda']

        pack_path = rospkg.RosPack().get_path('detic_ros')

        default_detic_config_path = os.path.join(
            pack_path, 'detic_configs',
            model_name + '.yaml')

        default_model_weights_path = os.path.join(
            pack_path, 'models',
            model_name + '.pth')

        return cls(
                enable_pubsub,
                out_debug_img,
                out_debug_segimg,
                verbose,
                vocabulary,
                custom_vocabulary,
                default_detic_config_path,
                default_model_weights_path,
                confidence_threshold,
                device_name)

    @classmethod
    def from_rosparam(cls):

        return cls.from_args(
                rospy.get_param('~enable_pubsub', True),
                rospy.get_param('~out_debug_img', True),
                rospy.get_param('~out_debug_segimg', False),
                rospy.get_param('~verbose', True),
                rospy.get_param('~confidence_threshold', 0.5),
                rospy.get_param('~device', 'auto'),
                rospy.get_param('~vocabulary', 'lvis'),
                rospy.get_param('~custom_vocabulary', ''),
                )

    def to_detectron_config(self):
        cfg = get_cfg()
        cfg.MODEL.DEVICE = self.device_name
        add_centernet_config(cfg)
        add_detic_config(cfg)
        cfg.merge_from_file(self.detic_config_path)
        cfg.MODEL.RETINANET.SCORE_THRESH_TEST = self.confidence_threshold
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = self.confidence_threshold
        cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = self.confidence_threshold
        cfg.merge_from_list(['MODEL.WEIGHTS', self.model_weights_path])

        # Similar to https://github.com/facebookresearch/Detic/demo.py
        cfg.MODEL.ROI_BOX_HEAD.ZEROSHOT_WEIGHT_PATH = 'rand' # load later
        cfg.MODEL.ROI_HEADS.ONE_CLASS_PER_PROPOSAL = True

        # Maybe should edit detic_configs/Base-C2_L_R5021k_640b64_4x.yaml
        pack_path = rospkg.RosPack().get_path('detic_ros')
        cfg.MODEL.ROI_BOX_HEAD.CAT_FREQ_PATH = os.path.join(
                pack_path, 'datasets/metadata/lvis_v1_train_cat_info.json')

        cfg.freeze()
        return cfg
