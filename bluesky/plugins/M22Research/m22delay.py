import bluesky as bs
import numpy as np

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'M22DELAY',
        'plugin_type': 'sim',
        'reset': M22Delay.reset
    }
    return config

class M22Delay(bs.core.Entity):
    def __init__(self):
        super().__init__()