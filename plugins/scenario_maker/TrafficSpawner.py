import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky.stack import command
import numpy as np

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'TRAFFICSPAWNER',
        'plugin_type': 'sim',
    }
    return config

class TrafficSpawner(Entity):
    def __init__(self):
        super().__init__()
        
    
    @command
    def DELETEALL(self):
        '''Deletes all aircraft.'''
        while self.ntraf>0:
            self.delete(0)
        return