import bluesky as bs
import numpy as np

from bluesky.traffic.asas import ConflictResolution

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'M22CR',
        'plugin_type': 'sim'
    }
    return config

class M22CR(ConflictResolution):
    def __init__(self):
        super().__init__()