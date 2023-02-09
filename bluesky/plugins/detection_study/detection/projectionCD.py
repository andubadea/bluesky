import bluesky as bs
from bluesky.tools.aero import nm
from bluesky.traffic.asas import ConflictDetection

import numpy as np
import shapely as sp
import math
import pyproj

from shapely.geometry import LineString, Point
from shapely import STRtree

def init_plugin():
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
        
        # Get the city centre
        try:
            self.city_centre_coords = bs.traf.TrafficSpawner.city_centre_coords
        except:
            print('City centre cannot be set, defaulting to Vienna.')
            self.city_centre_coords = [48.208758, 16.372449]
            
        # Get the UTM coordinate system
        utm_crs = self.convert_wgs_to_utm(*self.city_centre_coords)
        
        # Initialise the coordinate transformer
        self.transform_coords = pyproj.Transformer.from_crs(crs_from=4326, crs_to=utm_crs, always_xy = True)
        
        # Initialise the index
        self.geom_tree = self.create_index()
        self.rte_cut_geometries = []
        
    def reset(self):
        pass
    
    def clearconfdb(self):
        return super().clearconfdb()
    
    def update(self, ownship, intruder):
        # Update the geometry tree
        self.geom_tree = self.create_index()
        # Detect conficts
        self.detect(ownship, intruder)
    
    def detect(self, ownship, intruder):
        # Query the tree for now
        print(self.geom_tree.query(self.rte_cut_geometries))
    
    def create_index(self):
        """Function that creates the geometric tree index. 
        First process the current routes of aircraft, transform them into linestrings,
        reproject them to a 2D coordinate system, and create the index."""
        # Initialise the list of geometries
        self.rte_cut_geometries = []
        # We basically need to loop through all aircraft routes
        for acidx, acrte in enumerate(bs.traf.ap.route):
            # Pass this aircraft if it doesn't have a route
            if not acrte.wplat:
                continue
            
            # Convert the coordinates of the route to UTM
            rte_utm_lat = self.transform_coords.transform(acrte.wplat)
            rte_utm_lon = self.transform_coords.transform(acrte.wplon)
            
            # Create the linestring from the UTM coordinates
            rte_linestring = LineString(zip(rte_utm_lon, rte_utm_lat))
            
            # Cut the linestring in function of the lookahead time
            cut_dist = bs.traf.gs[acidx] + self.dtlookahead[acidx]
            cut_dist = max(min(cut_dist, self.lookahead_max), self.lookahead_min)
            
            cut_rte = self.cut(rte_linestring, cut_dist)
            
            # Add an acidx attribute to the linestring
            cut_rte.acidx = acidx
            
            # Add the route to the list of geometries
            self.rte_cut_geometries.append(cut_rte)
            
        # Create the index
        return STRtree(self.rte_cut_geometries)
        
    
    def convert_wgs_to_utm(self, lat: float, lon: float):
        """Based on lat and lng, return best utm epsg-code"""
        utm_band = str((math.floor((lon + 180) / 6 ) % 60) + 1)
        if len(utm_band) == 1:
            utm_band = '0'+utm_band
        if lat >= 0:
            epsg_code = '326' + utm_band
            return epsg_code
        epsg_code = '327' + utm_band
        return epsg_code
    
    def cut(self, line, distance):
        # Cuts a line in two at a distance from its starting point
        if distance <= 0.0 or distance >= line.length:
            return [LineString(line)]
        coords = list(line.coords)
        for i, p in enumerate(coords):
            pd = line.project(Point(p))
            if pd == distance:
                return LineString(coords[:i+1])
            if pd > distance:
                cp = line.interpolate(distance)
                return LineString(coords[:i] + [(cp.x, cp.y)])
    