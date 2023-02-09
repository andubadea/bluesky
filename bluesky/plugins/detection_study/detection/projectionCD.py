import bluesky as bs
import numpy as np
import shapely as sp
import math
import pyproj
import geopandas as gpd
import matplotlib.pyplot as plt

from shapely.geometry import LineString, Point
from shapely import STRtree
from bluesky.tools.aero import nm
from bluesky.traffic.asas import ConflictDetection

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
        all_intersections = np.transpose(self.geom_tree.query(self.rte_cut_geometries, predicate = 'intersects'))
        
        # Problem is, the query contains self intersections
        # We want the indices where column 0 is not equal to column 1
        acidx_int_pairs = all_intersections[all_intersections[:, 0] != all_intersections[:, 1]]
        
        print(acidx_int_pairs)
    
    def create_index(self):
        """Function that creates the geometric tree index. 
        First process the current routes of aircraft, transform them into linestrings,
        reproject them to a 2D coordinate system, and create the index."""
        # Initialise the list of geometries
        self.rte_cut_geometries = []
        # We basically need to loop through all aircraft routes
        for acidx, acrte in enumerate(bs.traf.ap.route):
            # Pass this aircraft if it doesn't have a route
            if not acrte.wplat or not acrte.wplon:
                continue
            
            # Get active waypoint
            act_wp = acrte.iactwp
            
            if act_wp < 0:
                # Weird thing
                continue
            
            # Add the current position of the aircraft to the lon and lat arrays
            ac_rte_lon = np.concatenate([[bs.traf.lon[acidx]], acrte.wplon[act_wp:]])
            ac_rte_lat = np.concatenate([[bs.traf.lat[acidx]], acrte.wplat[act_wp:]])
            
            # Convert the coordinates of the route to UTM
            rte_utm_lat,rte_utm_lon = self.transform_coords.transform(ac_rte_lon, ac_rte_lat)
            
            # Create the linestring from the UTM coordinates
            rte_linestring = LineString(zip(rte_utm_lon, rte_utm_lat))
            
            # Cut the linestring in function of the lookahead time
            cut_dist = bs.traf.gs[acidx] + self.dtlookahead[acidx]
            cut_dist = max(min(cut_dist, self.lookahead_max), self.lookahead_min)
            
            cut_rte = self.cut(rte_linestring, cut_dist)
            
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
    