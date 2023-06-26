import bluesky as bs
import numpy as np

from bluesky.core import Entity


def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'M22WIND',
        'plugin_type': 'sim',
        'reset': M22Wind.reset
    }
    return config

class M22Wind(Entity):
    def __init__(self):
        super().__init__()
        