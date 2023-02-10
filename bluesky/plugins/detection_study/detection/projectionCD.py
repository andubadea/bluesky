import bluesky as bs
import numpy as np
import shapely as sp
import math
import pyproj
import geopandas as gpd
import matplotlib.pyplot as plt

from shapely.geometry import LineString, Point, MultiLineString, MultiPoint, GeometryCollection
from shapely import STRtree
from shapely.ops import split, nearest_points
from bluesky.tools.aero import nm
from bluesky.traffic.asas import ConflictDetection
from bluesky.tools import geo

"""
This detection plugin only detects intersections in paths between aircraft, and
returns a list of LineStrings representing the intersections to be used in 
training a supervised learning detection and resolution model. 
"""

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
        
        # New detection parameters
        self.intent_geom = [] # Linestring of aircraft intent per pair
        self.dist_to_int = [] # Distance to intent intersections per pair
        self.num_turns = [] # Number of turns per pair
        self.mean_turn_angle = [] # Mean turn angle per pair
        self.qdr_mat = [] # QDR for all aircraft
        self.dist_mat = [] # Distance for all aircraft
        
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
        self.__init__()
        pass
    
    def clearconfdb(self):
        return super().clearconfdb()
    
    def update(self, ownship, intruder):
        # Update the geometry tree
        self.geom_tree = self.create_index()
        
        # Detect intersections
        self.confpairs, self.lospairs, self.inconf, self.dist_to_int, self.num_turns, \
        self. mean_turn_angle, self.qdr_mat, \
        self.dist_mat, self.intent_geom = self.detect(ownship, intruder)
                
        # confpairs has conflicts observed from both sides (a, b) and (b, a)
        # confpairs_unique keeps only one of these
        confpairs_unique = {frozenset(pair) for pair in self.confpairs}
        lospairs_unique = {frozenset(pair) for pair in self.lospairs}

        self.confpairs_all.extend(confpairs_unique - self.confpairs_unique)
        self.lospairs_all.extend(lospairs_unique - self.lospairs_unique)

        # Update confpairs_unique and lospairs_unique
        self.confpairs_unique = confpairs_unique
        self.lospairs_unique = lospairs_unique   
        return
    
    def detect(self, ownship, intruder):
        # Query the tree for now
        tree_query = self.geom_tree.query(self.rte_cut_geometries, predicate = 'intersects')
        all_intersections = np.transpose(tree_query)
        
        # Problem is, the query contains self intersections
        # We want the indices where column 0 is not equal to column 1
        acidx_int_pairs = all_intersections[all_intersections[:, 0] != all_intersections[:, 1]]
        
        # Get lospairs and inconf bools
        lospairs, qdr_mat, dist_mat = self.los_detect(ownship, intruder)
        inconf = np.array([False]*ownship.ntraf)
        
        if len(acidx_int_pairs) == 0:
            return [], [], inconf, [], [], [], qdr_mat, dist_mat, []
        
        inconf[acidx_int_pairs[0,:]] = True
        
        # For each confpair, we need to get the distance of each aircraft to the intersection,
        # the number of turns until the intersection, and the mean turn angle until the intersection
        # First, initialize the arrays

        dist_to_int = [-999,-999] * len(acidx_int_pairs)
        num_turns = [-999,-999] * len(acidx_int_pairs)
        mean_turn_angle = [-999,-999] * len(acidx_int_pairs)
        intent_geom = [-999,-999] * len(acidx_int_pairs)
        
        for j, pair in enumerate(acidx_int_pairs):
            idx1, idx2 = pair[0], pair[1]
            # Get info from the functions
            dist1, dist2, split1, split2, int_point = self.distance_to_intersection(idx1, idx2)
            num_turns1, num_turns2, mean_turn_angle1, mean_turn_angle2 = self.turn_info(idx1, idx2, split1, split2)
            
            # Assign the values
            dist_to_int[j] = [dist1, dist2]
            num_turns[j] = [num_turns1, num_turns2]
            mean_turn_angle[j] = [mean_turn_angle1, mean_turn_angle2]
            intent_geom[j] = [split1, split2]
            
            # Verification
            if num_turns1 > 0 or num_turns2 > 0:
                print('----------------------------------------------------------------')
                print(bs.traf.id[idx1], bs.traf.id[idx2])
                print(dist1, dist2)
                print(num_turns1, num_turns2)
                print(mean_turn_angle1, mean_turn_angle2)
                plt.plot(split1.coords.xy[0], split1.coords.xy[1])
                plt.plot(split2.coords.xy[0], split2.coords.xy[1])
                plt.scatter(int_point.x, int_point.y)
                plt.show(block = True)
        
        # Change to strings
        acidx_int_pairs = [(bs.traf.id[pair[0]], bs.traf.id[pair[1]]) for pair in acidx_int_pairs]
        
        return acidx_int_pairs, lospairs, inconf, dist_to_int, num_turns, mean_turn_angle, qdr_mat, dist_mat, intent_geom
    
    def distance_to_intersection(self, idx1, idx2):
        """Function that calculates the distance to the intersection between two aircraft.
        """
        # Get the geometries of the aircraft
        intent1 = self.rte_cut_geometries[idx1]
        intent2 = self.rte_cut_geometries[idx2]
        
        # Get the intersection point between the two intents
        int_point = intent1.intersection(intent2)
        
        # Make this thing a point no matter what
        # Basically, always take the first point
        if isinstance(int_point, LineString):
            int_point = Point(int_point.coords[0])
            
        if isinstance(int_point,MultiLineString):
            int_point = Point(int_point.geoms[0].coords[0])
        
        if isinstance(int_point, MultiPoint):
            int_point = int_point.geoms[0]

        if isinstance(int_point, GeometryCollection):
            int_point = int_point.geoms[0]

            if isinstance(int_point, LineString):
                int_point = Point(int_point.coords[0])
            
        # Split the lines at the intersection point
        intent1_split = split(intent1, int_point).geoms[0]
        intent2_split = split(intent2, int_point).geoms[0]
        # Return the lenghts
        return intent1_split.length, intent2_split.length, intent1_split, intent2_split, int_point
    
    def turn_info(self, idx1, idx2, split1, split2):
        """Function that calculates the number of turns to the intersection between two aircraft.
        """
        # Not all points within the linestring are turns, only if the angle is greater than 25 degrees.
        # Thus, get all the angles above an absolute value of 25 degrees.
        # We can determine this stuff by checking the turn waypoints of aircraft 
        # and whether they belong to the line segment between the aircraft and the 
        # intersection.
        
        # Get the routes
        acrte1 = bs.traf.ap.route[idx1]
        acrte2 = bs.traf.ap.route[idx2]
        
        # Active waypoints
        act_wp1 = acrte1.iactwp
        act_wp2 = acrte2.iactwp
        
        # Turn waypoints
        turnidx1 = np.where(acrte1.wpflyturn)[0]
        turnidx2 = np.where(acrte2.wpflyturn)[0]
        
        # We can determine if there are any turn waypoints between the aircraft and the intersection
        # by simply finding out the index of the waypoint right before the intersection and seeing
        # if any waypoints in betweeen are turn waypoints.
        
        # Get the index of the waypoint right before the intersection
        wptidx_b4_int1 = act_wp1 + (len(split1.coords) - 2)
        wptidx_b4_int2 = act_wp2 + (len(split2.coords) - 2)
        
        # Calculate the number of turns
        num_turns_1 = np.sum(np.logical_and(turnidx1 > act_wp1, turnidx1 < wptidx_b4_int1))
        num_turns_2 = np.sum(np.logical_and(turnidx2 > act_wp2, turnidx2 < wptidx_b4_int2))
        
        # Now that we know the number of turns, calculate the mean turn angle
        # We need to for loop through all the turn waypoints for each aircraft
        angles1 = []
        for i_turn in turnidx1[np.logical_and(turnidx1 > act_wp1, turnidx1 < wptidx_b4_int1)]:
            if not (i_turn < len(acrte1.wplon)-1):
                continue
            # Get the needed stuff
            lat_cur, lon_cur   = acrte1.wplat[i_turn], acrte1.wplon[i_turn]
            lat_prev, lon_prev = acrte1.wplat[i_turn-1], acrte1.wplon[i_turn-1]
            lat_next, lon_next = acrte1.wplat[i_turn+1], acrte1.wplon[i_turn+1]
            
            # Get the angle
            d1=geo.kwikqdrdist(lat_prev,lon_prev,lat_cur,lon_cur)
            d2=geo.kwikqdrdist(lat_cur,lon_cur,lat_next,lon_next)
            angle=abs(d2[0]-d1[0])

            if angle>180:
                angle=360-angle
                
            # This is a turn if angle is greater than 25
            if angle > 25:
                angles1.append(angle)
            else:
                continue
            
        angles2 = []
        for i_turn in turnidx2[np.logical_and(turnidx2 > act_wp2, turnidx2 < wptidx_b4_int2)]:
            if not (i_turn < len(acrte2.wplon)-1):
                continue
            # Get the needed stuff
            lat_cur, lon_cur   = acrte2.wplat[i_turn], acrte2.wplon[i_turn]
            lat_prev, lon_prev = acrte2.wplat[i_turn-1], acrte2.wplon[i_turn-1]
            lat_next, lon_next = acrte2.wplat[i_turn+1], acrte2.wplon[i_turn+1]
            
            # Get the angle
            d1=geo.kwikqdrdist(lat_prev,lon_prev,lat_cur,lon_cur)
            d2=geo.kwikqdrdist(lat_cur,lon_cur,lat_next,lon_next)
            angle=abs(d2[0]-d1[0])

            if angle>180:
                angle=360-angle
                
            # This is a turn if angle is greater than 25
            if angle > 25:
                angles2.append(angle)
            else:
                continue
            
        avg_angle1 = np.average(angles1) if len(angles1) > 0 else 0
        avg_angle2 = np.average(angles2) if len(angles2) > 0 else 0
        
        # Return the number of turns
        return num_turns_1, num_turns_2, avg_angle1, avg_angle2
    
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
            
            # Find the position of the aircraft on the current leg
            # TODO: This
            
            # Add the current position of the aircraft to the lon and lat arrays
            ac_rte_lon = np.concatenate([[bs.traf.lon[acidx]], acrte.wplon[act_wp:]])
            ac_rte_lat = np.concatenate([[bs.traf.lat[acidx]], acrte.wplat[act_wp:]])
            
            # Convert the coordinates of the route to UTM
            rte_utm_lat,rte_utm_lon = self.transform_coords.transform(ac_rte_lon, ac_rte_lat)
            
            # Create the linestring from the UTM coordinates
            rte_linestring = LineString(zip(rte_utm_lat, rte_utm_lon))
            
            # Cut the linestring in function of the lookahead time
            cut_dist = bs.traf.gs[acidx] + self.dtlookahead[acidx]
            cut_dist = max(min(cut_dist, self.lookahead_max), self.lookahead_min)
            
            cut_rte = self.cut(rte_linestring, cut_dist)
            
            # Add the route to the list of geometries
            self.rte_cut_geometries.append(cut_rte)
            
        # Create the index
        return STRtree(self.rte_cut_geometries)
    
    def los_detect(self, ownship, intruder):
        ''' Intrusion between ownship (traf) and intruder (traf/adsb).'''
        # Identity matrix of order ntraf: avoid ownship-ownship detected conflicts
        I = np.eye(ownship.ntraf)

        qdr, dist = geo.kwikqdrdist_matrix(np.asmatrix(ownship.lat), np.asmatrix(ownship.lon),
                                    np.asmatrix(intruder.lat), np.asmatrix(intruder.lon))

        qdr = np.asarray(qdr)
        dist = np.asarray(dist) * nm + 1e9 * I

        dalt = ownship.alt.reshape((1, ownship.ntraf)) - \
            intruder.alt.reshape((1, ownship.ntraf)).T  + 1e9 * I

        swlos = (dist < (np.zeros(len(self.rpz)) + self.rpz)) * (np.abs(dalt) < self.hpz)
        lospairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swlos))]

        return lospairs, qdr, dist
        
    
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
            return LineString(line)
        coords = list(line.coords)
        for i, p in enumerate(coords):
            pd = line.project(Point(p))
            if pd == distance:
                return LineString(coords[:i+1])
            if pd > distance:
                cp = line.interpolate(distance)
                return LineString(coords[:i] + [(cp.x, cp.y)])
    