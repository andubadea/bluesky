import bluesky as bs
from bluesky.tools.aero import nm
from bluesky.traffic.asas import ConflictDetection

import numpy as np
import shapely as sp
import math

def init_plugin():

    # Addtional initilisation code

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'PROJECTIONCD',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

class ProjectionCD(ConflictDetection):
    def __init__(self):
        super().__init__()
        
        self.lookahead_min = 100 #metres
        self.lookahead_max = 300 #metres
        
    def reset(self):
        pass
    
    def clearconfdb(self):
        return super().clearconfdb()
    
    def update(self):
        pass
    
    
    def create_index(self):
        """Function that creates the geometric tree index. 
        First process the current routes of aircraft, transform them into linestrings,
        reproject them to a 2D coordinate system, and create the index."""
        pass
    
    @staticmethod
    def convert_wgs_to_utm(lon: float, lat: float):
        """Based on lat and lng, return best utm epsg-code"""
        utm_band = str((math.floor((lon + 180) / 6 ) % 60) + 1)
        if len(utm_band) == 1:
            utm_band = '0'+utm_band
        if lat >= 0:
            epsg_code = '326' + utm_band
            return epsg_code
        epsg_code = '327' + utm_band
        return epsg_code
    