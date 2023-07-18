"""This plugin holds the wind model for each wind simulation.
At the beginning of a simulation, this plugin generates a wind value and
direction for each and every edge within the graph, with a set mean and variance.
"""
import bluesky as bs
import numpy as np

from bluesky.core import Entity


def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'M22WIND',
        'plugin_type': 'sim',
        'reset': reset
    }
    bs.traf.m22wind = M22Wind()
    return config

def reset():
    bs.traf.m22wind.reset()

class M22Wind(Entity):
    def __init__(self):
        super().__init__()
        
    def reset(self):
        pass
        