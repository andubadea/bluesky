"""This plugin holds the wind model for each wind simulation.
At the beginning of a simulation, this plugin generates a wind value and
direction for each and every edge within the graph, with a set mean and variance.
"""
import bluesky as bs
import numpy as np

from bluesky.tools.aero import kts
from bluesky.traffic.windsim import WindSim


def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'M22WIND',
        'plugin_type': 'sim',
    }
    return config


class M22Wind(WindSim):
    def __init__(self):
        super().__init__()
        # This will contain magnitudes for each street. Street index is equivalent to array index
        self.magnitudes = np.array([]) #m/s
        # 1 if aligned with street direction (speeds up aircraft)
        # -1 if misaligned with street direction (slows down aircraft)
        self.direction = np.array([])
        # Create the wind
        self.create_wind()
        
    def create_wind(self):
        '''Creates the wind model on the fly. 
        Also caps the wind magnitude to a maximum of 8 m/s according to the M600 limits.'''
        pass
    
    def get_street_ids(self):
        '''Because there is no trafarray with iactwp, this needs to be a for loop.'''
        # Get the active waypoint indices
        iactwps = [route.iactwp for route in bs.traf.ap.route]
        # Now return the street indices on which aircraft is in function of iactwp
        return [bs.traf.TrafficHandler.street_numbers[i][j] for i,j in enumerate(iactwps)]
        
    
    def getdata(self, lats, lons, alts):
        '''This function needs to return vnorth and veast. Thus, we ignore the lats, lons, alts
        and just return the individual wind for each and every aircraft.'''
        # Exception in case we don't have vectors yet. If we only have one aircraft, wind doesn't
        # matter anyway
        if len(lats) == 1 and bs.traf.gs[-1] == 0:
            return 0,0
        # Convert to radians
        hdg = np.deg2rad(bs.traf.hdg)
        # Basically, the absolute value of the wind will be affected for each and every
        # aircraft that is a following a street. 
        # This, we first take the wind value for each and every aircraft in function of
        # the street they are on
        street_ids = self.get_street_ids()
        
        # Now get the magnitudes and directions of the wind. Positive if speed increases, negative if speed decreases.
        gs_windmags = self.magnitudes[street_ids] * self.direction[street_ids]
        
        # Now get the would-be wind-inclusive ground speed magnitudes
        gs_would_be = bs.traf.gs + gs_windmags
        
        # Now project these onto the direction of the aircraft
        gs_would_be_east = gs_would_be * np.sin(hdg)
        gs_would_be_north = gs_would_be * np.cos(hdg)
        
        # Now also get the normal ground speeds
        gseast = bs.traf.gs * np.sin(hdg)
        gsnorth = bs.traf.gs * np.cos(hdg)
        
        # The wind to return is just the difference between these
        veast = gs_would_be_east - gseast
        vnorth = gs_would_be_north - gsnorth

        # Aircraft going slow are unaffected
        veast = np.where(bs.traf.gs < 10*kts, 0, veast)
        vnorth = np.where(bs.traf.gs < 10*kts, 0 , vnorth)
        
        return vnorth, veast
        