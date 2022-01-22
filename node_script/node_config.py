import os
import dataclasses
import rospy
import rospkg

@dataclasses.dataclass
class NodeConfig:
    out_debug_img: bool
    out_debug_segimage: bool
    verbose: bool
    voabulary: str
    detic_config_path: str
    model_weights_path: str
    confidence_threshold: float

    @classmethod
    def from_rosparam(cls):
        pack_path = rospkg.RosPack().get_path('detic_ros')

        default_detic_config_path = os.path.join(
            pack_path, 'detic_configs', 
            'Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml')

        default_model_weights_path = os.path.join(
            pack_path, 'models',
            'Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth')

        return cls(
                rospy.get_param('~out_debug_img', True),
                rospy.get_param('~out_debug_segimage', False),
                rospy.get_param('~verbose', False),
                rospy.get_param('~vocabulary', 'lvis'),
                rospy.get_param('~detic_config_path', default_detic_config_path),
                rospy.get_param('~model_weights_path', default_model_weights_path),
                rospy.get_param('~confidence_threshold', 0.5))
