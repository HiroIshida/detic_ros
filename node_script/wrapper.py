
class DeticWrapper:

    def __init__(self, node_config: Optional[NodeConfig]=None):
        if node_config is None:
            node_config = NodeConfig.from_rosparam()
