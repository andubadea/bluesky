"""This plugin holds the wind model for each wind simulation.
At the beginning of a simulation, this plugin generates a wind value and
direction for each and every edge within the graph, with a set mean and variance.
"""
import bluesky as bs
import numpy as np
import geopandas as gpd

from bluesky.tools.aero import kts
from bluesky.traffic.windsim import WindSim
from bluesky import stack
from bluesky.tools.geo import kwikqdrdist


def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'CDRWIND',
        'plugin_type': 'sim',
    }
    return config


class CDRWind(WindSim):
    def __init__(self):
        super().__init__()
        # Global wind properties
        self.global_mag = 0
        self.global_dir = 0
        # Load the streets
        self.streets_gpd, self.streets_bearings = self.load_steets('Vienna')
        # This will contain magnitudes for each street. Street index is equivalent to array index
        self.magnitudes = np.zeros(len(self.streets_gpd)) #m/s
        # 1 if aligned with street direction (speeds up aircraft)
        # -1 if misaligned with street direction (slows down aircraft)
        self.directions = np.zeros(len(self.streets_gpd))
        # Create the wind
        self.create_wind()
        
    @stack.command
    def setwind(self, magnitude:float, direction:float):
        '''Set the magnitude [m/s] and direction [deg] of the wind.
        The direction is where the wind is blowing towards, i.e., if blowing from West towards
        East, dir needs to be 90 deg.'''
        self.global_mag = magnitude
        self.global_dir = direction
        # Disable if the magnitude is set to 0
        if magnitude == 0:
            self.winddim = 0
            return
        
        self.create_wind()
        # Set the winddim at more than 0
        self.winddim = 1

    def load_steets(self, city):
        '''Load the streets of the city and compute their average bearings.'''      
        streets_gpd = gpd.read_file(f'bluesky/plugins/detection_study/scenario_maker/{city}/street_groups.gpkg')
        streets_bearings = [0] * len(streets_gpd)
        # The bearings will be a weighted average of all the bearings in the linestring of each street
        for street_no in range(len(streets_gpd)):
            # Extract the linestring
            line = streets_gpd.loc[street_no]['geometry']
            # Get the weighted bearing of this
            streets_bearings[street_no] = self.weighted_bearing(line)
        return streets_gpd, streets_bearings
    
    def get_current_edges(self):
        # Get all the edges for all aircraft
        edges_trafarray = bs.traf.TrafficSpawner.route_edges
        #Initialise the edges
        current_edge = [None] * bs.traf.ntraf
        # We basically need to loop through all aircraft routes
        for acidx, acrte in enumerate(bs.traf.ap.route):
            # Skip aircraft if it doesn't have a route
            if len(acrte.wplat) == 0:
                continue
            
            current_edge[acidx] = edges_trafarray[acidx][acrte.iactwp]
        return current_edge

    def weighted_bearing(self, line):
        '''Calculates the weighted bearing of a Shapely Linestring'''
        # Extract the coords, they will be lon/lat
        coords = list(line.coords)
        # Get the bearings and segment lengths
        temp = np.array([kwikqdrdist(coords[i][1], coords[i][0], 
                                           coords[i+1][1], coords[i+1][0]) for i in range(len(coords)-1)])
        
        # Get the weighted average bearing
        return np.sum(temp[:,0] * temp[:,1])/np.sum(temp[:,1])
        
    def create_wind(self):
        '''Creates the wind model on the fly from a street graph. 
        Also caps the wind magnitude to a maximum of 8 m/s according to the M600 limits.'''
        # First of all, if the global magnitude is 0, then don't do wind
        if self.global_mag == 0:
            return

        for i in range(len(self.streets_bearings)):
            # i is also the street number
            bearing = self.streets_bearings[i]
            # Get the wind magnitude and direction
            self.magnitudes[i], self.directions[i] = self.wind_from_bearing(bearing)
            
    def wind_from_bearing(self, bearing):
        '''Give the wind bearing and direction in function of street bearing.'''
        # First of all, we can get a reference wind magnitude projected on this bearing
        angle_diff = self.get_anglular_distance(bearing, self.global_dir)
        # If the difference is greater than 90, then by default wind goes against the direction
        # of the street. Else, the wind is by default along the street
        if angle_diff > 90:
            default_dir = -1
        else:
            default_dir = 1
        
        # Also get the default magnitude of the wind on this street
        default_mag = abs(np.cos(np.deg2rad(angle_diff)) * self.global_mag)
        
        # We can introduce some randomness in the wind by sampling a normal distribution around the
        # default magnitude. Absolute value in case it somehow samples a negative one.
        sampled_mag = abs(np.random.normal(default_mag, default_mag/10))
        
        # If the angle difference is smaller than 30, then it is guaranteed that the direction
        # matches the global
        if angle_diff < 30:
            return sampled_mag, default_dir
        
        # Otherwise, we first make a roll to see if we should flip a coin or not. This probability is
        # just the sin value
        coin_flip_probability = abs(np.sin(np.deg2rad(angle_diff)))
        if np.random.random() < coin_flip_probability:
            # Perform a coin flip
            if np.random.random() < 0.5:
                # Flip the direction
                default_dir *= -1
        
        # Return the magnitude and direction
        return sampled_mag, default_dir
        
    
    def get_anglular_distance(self, unit1, unit2):
        '''Returns the absolute angular distance between two angles in degrees.'''
        phi = abs(unit2-unit1) % 360
        if phi > 180:
            return 360-phi
        else:
            return phi
    
    def get_street_ids(self):
        '''Because there is no trafarray with iactwp, this needs to be a for loop.'''
        # Get the current edges
        current_edges = self.get_current_edges()
        # Copy the edges over
        street_dict = bs.traf.TrafficSpawner.street_dict
        # Convert these edges to street numbers
        street_numbers = [street_dict[tuple(edge)] for edge in current_edges]
        # Now return the street indices on which aircraft is in function of iactwp
        return street_numbers
        
    
    def getdata(self, lats, lons, alts):
        '''This function needs to return vnorth and veast. Thus, we ignore the lats, lons, alts
        and just return the individual wind for each and every aircraft.'''
        # If global magnitude is 0, just return 0s
        if self.global_mag == 0:
            return np.zeros(len(lats)), np.zeros(len(lats))
        
        if len(lats) == 1:
            # This getdata is coming from a CRE command, so ignore it
            return 0,0
    
        # Convert to radians
        hdg = np.deg2rad(bs.traf.hdg)
        # Basically, the absolute value of the wind will be affected for each and every
        # aircraft that is a following a street. 
        # This, we first take the wind value for each and every aircraft in function of
        # the street they are on
        street_ids = self.get_street_ids()
        
        # Now get the magnitudes and directions of the wind. Positive if speed increases, negative if speed decreases.
        gs_windmags = self.magnitudes[street_ids] * self.directions[street_ids]
        
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

        # Aircraft going slower than 15 kts are unaffected.
        veast = np.where(bs.traf.tas < 15*kts, 0, veast)
        vnorth = np.where(bs.traf.tas < 15*kts, 0 , vnorth)
        
        return vnorth, veast
        