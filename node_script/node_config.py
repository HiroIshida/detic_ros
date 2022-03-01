import os
from dataclasses import dataclass
import rospy
import rospkg

import torch


@dataclass
class NodeConfig:
    out_debug_img: bool
    out_debug_segimage: bool
    verbose: bool
    voabulary: str
    detic_config_path: str
    model_weights_path: str
    confidence_threshold: float
    device_name: str

    @classmethod
    def from_args(cls, 
            out_debug_img: bool = True,
            out_debug_segimage: bool = True,
            verbose: bool = False,
            confidence_threshold: float = 0.5,
            device_name: str = 'cpu',
            ):

        if device_name == 'auto':
            device_name = 'cuda' if torch.cuda.is_available() else 'cpu'

        assert device_name in ['cpu', 'cuda']

        pack_path = rospkg.RosPack().get_path('detic_ros')

        default_detic_config_path = os.path.join(
            pack_path, 'detic_configs', 
            'Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml')

        default_model_weights_path = os.path.join(
            pack_path, 'models',
            'Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth')

        return cls(
                out_debug_img,
                out_debug_segimage,
                verbose,
                'lvis',
                default_detic_config_path,
                default_model_weights_path,
                confidence_threshold,
                device_name)

    @classmethod
    def from_rosparam(cls):

        return cls.from_args(
                rospy.get_param('~out_debug_img', True),
                rospy.get_param('~out_debug_segimage', False),
                rospy.get_param('~verbose', True),
                rospy.get_param('~confidence_threshold', 0.5),
                rospy.get_param('~device', 'auto'),
                )
